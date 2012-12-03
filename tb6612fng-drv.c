#include <linux/module.h>
#include <linux/gpio.h>
#include <rtdm/rtdm_driver.h>
#include "gpio-manip.h"
#include "pwm-task-proc.h"
#include "interface.h"
#include "pid-controller.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrey Nechypurenko <andreynech@gmail.com>");
MODULE_DESCRIPTION("TB6612FNG dual motor controller driver");

#define N_ENCODERS 2

typedef struct tagEncoderInfo
{
  rtdm_irq_t irq_handles[2];// A and B channels
  u8 a_pin; // encoder input A
  u8 b_pin; // encoder input B
  u8 prev_value; // gpio values for direction detection
  u8 cur_value;
  u16 target_speed; // in state changes within DELTA_T
  nanosecs_abs_t prev_time;
  u8 direction;
  nanosecs_abs_t cur_time;
  u16 state_changes_counter;
  u16 prev_state_changes_counter;
  u32 states_to_go;
  u8 reported;
} EncoderInfo;


/**
 * The context of a device instance
 *
 * A context is created each time a device is opened and passed to
 * other device handlers when they are called.
 *
 */
typedef struct tb6612fng_context_s
{
  rtdm_lock_t lock; // lock to protect this context struct
  rtdm_event_t move_done_event; // event to wake up readers
  EncoderInfo enc_info[N_ENCODERS];
  pid_instance_params_t pid_params; // PID controller configuration
  long unsigned int pid_err[N_ENCODERS]; // PID cummulative square error
  int current_pwm_duties[N_ENCODERS];
  rtdm_task_t pid_task;
  u8 pid_task_should_stop;
} tb6612fng_context_t;


// Info to register IRQs for encoders.
// Used in driver's open() function.
typedef struct irq_request_s
{
  u8 gpio_pin;
  char *description;
} irq_request_t;

static const irq_request_t irq_request_info[N_ENCODERS][2] = 
{
  {
    { ENC1A, "wheel-encoder-1A" },
    { ENC1B, "wheel-encoder-1B" }
  },
  {
    { ENC2A, "wheel-encoder-2A" },
    { ENC2B, "wheel-encoder-2B" }
  }
};


// Quadrature Encoder Matrix.
// Please refer the following link for more details:
// http://letsmakerobots.com/node/24031
static const u8 QEM[16] = 
{ // curr value
   0,-1, 1, 2, // p
   1, 0, 2,-1, // r
  -1, 2, 0, 1, // e
   2, 1,-1, 0  // v
};


void 
tb6612fng_cleanup_ctx(tb6612fng_context_t *ctx)
{
  rtdm_event_destroy(&ctx->move_done_event);
}


int 
encoder_irq_handler(rtdm_irq_t *irq_context)
{
  tb6612fng_context_t *ctx;
  size_t i;

  ctx = rtdm_irq_get_arg(irq_context, tb6612fng_context_t);

  // Find out which encoder has triggered the interrupt
  for(i = 0; i < ARRAY_SIZE(ctx->enc_info); ++i)
    if(irq_context == &(ctx->enc_info[i].irq_handles[0])
       || irq_context == &(ctx->enc_info[i].irq_handles[1]))
      break;

  rtdm_lock_get(&ctx->lock);

  ++ctx->enc_info[i].state_changes_counter;

  ctx->enc_info[i].prev_value = ctx->enc_info[i].cur_value;
  // cur_value = inputA * 2 + inputB
  ctx->enc_info[i].cur_value = 
    gpio_get_value(ctx->enc_info[i].a_pin) * 2 + gpio_get_value(ctx->enc_info[i].b_pin);
  ctx->enc_info[i].direction = 
    QEM[ctx->enc_info[i].prev_value * 4 + ctx->enc_info[i].cur_value];

  if(ctx->enc_info[i].states_to_go == 0)
      return RTDM_IRQ_HANDLED;

  if(ctx->enc_info[i].states_to_go == 1)
    {
      ctx->enc_info[i].states_to_go = 0;
      ctx->enc_info[i].reported = 0;
      ctx->enc_info[i].target_speed = 0;
      //setmotorduty(i, 0);
      //rtdm_event_pulse(&ctx->move_done_event);
      rtdm_event_signal(&ctx->move_done_event);
    }
  else
    --ctx->enc_info[i].states_to_go;

  rtdm_lock_put(&ctx->lock);

  return RTDM_IRQ_HANDLED;
}


void 
pid_task_proc(void *arg)
{
  tb6612fng_context_t *ctx = (tb6612fng_context_t*)arg;
  rtdm_lockctx_t lock_ctx;
  size_t i;
  int crosstrack_error;
  int target_value = 0;
  int actual_value;
  int control;
  int duty;

  while(!ctx->pid_task_should_stop)
    {
      rtdm_task_wait_period();
      /*
      retcode = rtdm_task_wait_period();
      if(0 != retcode)
        rtdm_printk("TB6612FNG: rtdm_task_wait_period(): error %i\n",
		    retcode);
      */
      for(i = 0; i < N_ENCODERS; ++i)
	{
	  rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

	  ctx->enc_info[i].prev_time = ctx->enc_info[i].cur_time;
	  ctx->enc_info[i].cur_time = rtdm_clock_read_monotonic();
	  ctx->enc_info[i].prev_state_changes_counter = 
	    ctx->enc_info[i].state_changes_counter;
	  ctx->enc_info[i].state_changes_counter = 0;

	  target_value = ctx->enc_info[i].target_speed;
	  actual_value = ctx->enc_info[i].prev_state_changes_counter;

	  rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

	  if(target_value == 0) 
	    {
	      if(ctx->current_pwm_duties[i] != 0)
		{
		  ctx->current_pwm_duties[i] = 0;
		  setmotorduty(i, 0);
		}
	    }
	  else
	    {
	      control = pidcalc(&ctx->pid_params[i], 
				target_value, 
				actual_value);
	      duty = ctx->current_pwm_duties[i] + control;
	      // Setting duty to 100% causes periodicall
	      // rtdm_task_wait_period() -ETIMEDOUT errors on maximum
	      // rotation speed. Do not know why it happens, but I
	      // think it is related to timers management in pwm
	      // tasks. Need to investigate.
	      if(duty > 99)
		duty = 99;
	      else if(duty < 1)
		duty = 1;
	      if(ctx->current_pwm_duties[i] != duty)
		{
		  ctx->current_pwm_duties[i] = duty;
		  /*
		    rtdm_printk("PID %u: c:%i t:%i a:%i d:%i\n",
		    i,
		    control,
		    target_value,
		    actual_value,
		    duty);
		  */
		  setmotorduty(i, duty);
		}

	      // Collect crosstrack error for twiddle() and other
	      // statistic purposes
	      crosstrack_error = target_value - actual_value;
	      ctx->pid_err[i] += crosstrack_error * crosstrack_error;
	      /*
	      rtdm_printk("Crosstrack error: %i, square: %i, cummulative %lu\n", 
			  crosstrack_error, crosstrack_error * crosstrack_error, ctx->pid_err[i]);
	      */
	    }
	}
    }
}


/**
 * Open the device
 *
 * This function is called when the device shall be opened.
 *
 */
static int 
pwm_rtdm_open_nrt(struct rtdm_dev_context *context,
		  rtdm_user_info_t *user_info, 
		  int oflags)
{
  size_t i;
  int err;
  tb6612fng_context_t *ctx = (tb6612fng_context_t*)context->dev_private;  

  rtdm_lock_init(&ctx->lock);
  rtdm_event_init(&ctx->move_done_event, 0);

  for(i = 0; i < ARRAY_SIZE(ctx->enc_info); ++i)
    {
      ctx->enc_info[i].cur_value = 0;
      ctx->enc_info[i].prev_value = 0;
      ctx->enc_info[i].direction = 0;
      ctx->enc_info[i].target_speed = 0;
      ctx->enc_info[i].prev_time = 0;
      ctx->enc_info[i].cur_time = 0;
      ctx->enc_info[i].states_to_go = 0;
      ctx->enc_info[i].reported = 1;

      ctx->enc_info[i].a_pin = irq_request_info[i][0].gpio_pin;
      ctx->enc_info[i].b_pin = irq_request_info[i][1].gpio_pin;

      ctx->enc_info[i].prev_time = rtdm_clock_read_monotonic();
      ctx->enc_info[i].prev_state_changes_counter = 0;
      ctx->enc_info[i].state_changes_counter = 0;

      // Register IRQ for i-th encoder's channel A (0)
      err = rtdm_irq_request(&(ctx->enc_info[i].irq_handles[0]), 
			     gpio_to_irq(irq_request_info[i][0].gpio_pin),
			     encoder_irq_handler, 
			     RTDM_IRQTYPE_SHARED | RTDM_IRQTYPE_EDGE, 
			     irq_request_info[i][0].description, 
			     ctx);
      if(err)
	{
	  rtdm_printk("TB6612FNG: error requesting %s irq: %i\n",
		      irq_request_info[i][0].description,
		      err);
	  tb6612fng_cleanup_ctx(ctx);
	  return err;
	}
      else
	rtdm_printk("TB6612FNG: irq %u for %s requested\n",
		    gpio_to_irq(irq_request_info[i][0].gpio_pin),
		    irq_request_info[i][0].description);

      // Register IRQ for i-th encoder's channel B (1)
      err = rtdm_irq_request(&ctx->enc_info[i].irq_handles[1], 
			     gpio_to_irq(irq_request_info[i][1].gpio_pin),
			     encoder_irq_handler, 
			     RTDM_IRQTYPE_SHARED | RTDM_IRQTYPE_EDGE, 
			     irq_request_info[i][1].description, 
			     ctx);
      if(err)
	{
	  rtdm_printk("TB6612FNG: error requesting %s irq: %i\n",
		      irq_request_info[i][1].description,
		      err);
	  tb6612fng_cleanup_ctx(ctx);
	  return err;
	}
      else
	rtdm_printk("TB6612FNG: irq %u for %s requested\n",
		    gpio_to_irq(irq_request_info[i][1].gpio_pin),
		    irq_request_info[i][1].description);
    }

  ctx->pid_task_should_stop = 0;
  for(i = 0; i < ARRAY_SIZE(ctx->pid_params); ++i)
    {
      ctx->pid_params[i].max = 5;
      ctx->pid_params[i].min = -ctx->pid_params[i].max;
      // Experimentally found reasonable values for K
      ctx->pid_params[i].K[0] = 2380;
      ctx->pid_params[i].K[1] = 0;
      ctx->pid_params[i].K[2] = 1736;
      ctx->pid_params[i].prev_error = 0;
      ctx->pid_params[i].integral = 0;
      ctx->pid_err[i] = 0;
      ctx->current_pwm_duties[i] = 0;
    }
  err = rtdm_task_init(&ctx->pid_task, 
		       "pid-task",
		       pid_task_proc,
		       ctx,
		       RTDM_TASK_HIGHEST_PRIORITY + RTDM_TASK_LOWER_PRIORITY,
		       DELTA_T * 1000000);
  if(err)
    {
      rtdm_printk("PWM: error creating RTDM task: %i\n", err);
      return err;
    }

  return 0;
}

/**
 * Close the device
 *
 * This function is called when the device shall be closed.
 *
 */
static int 
pwm_rtdm_close_nrt(struct rtdm_dev_context *context,
		   rtdm_user_info_t * user_info)
{
  tb6612fng_context_t *ctx = (tb6612fng_context_t*)context->dev_private;  
  size_t i;

  // Unregister A and B IRQ handlers for all encoders
  for(i = 0; i < ARRAY_SIZE(ctx->enc_info); ++i)
    {
      rtdm_irq_free(&(ctx->enc_info[i].irq_handles[0]));
      rtdm_irq_free(&(ctx->enc_info[i].irq_handles[1]));
      ctx->enc_info[i].target_speed = 0;
    }

  ctx->pid_task_should_stop = 1;
  rtdm_task_sleep(2 * DELTA_T * 1000000);
  rtdm_task_destroy(&ctx->pid_task);

  for(i = 0; i < ARRAY_SIZE(ctx->enc_info); ++i)
    {
      setmotorduty(i, 0);
      turnmotor(i, 0);
    }

  tb6612fng_cleanup_ctx(ctx);

  return 0;
}

/**
 * Read from the device
 *
 * This function is called when the device is read in non-realtime
 * context.
 *
 */
static ssize_t 
pwm_rtdm_read(struct rtdm_dev_context *context,
		  rtdm_user_info_t *user_info, 
		  void *buf,
		  size_t nbyte)
{
  tb6612fng_context_t *ctx = (tb6612fng_context_t*)context->dev_private;  
  static const size_t n_enc = ARRAY_SIZE(ctx->enc_info);
  size_t i = n_enc;
  rtdm_lockctx_t lock_ctx;
  ControlCommandResponse response;

  if(nbyte == 0)
    return 0;

  if(user_info && !rtdm_read_user_ok(user_info, buf, nbyte))
    return -EFAULT;
  
  do
    {
      rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
      for(i = 0; i < n_enc; ++i)
	{
	  if(ctx->enc_info[i].reported == 0 
	     && ctx->enc_info[i].states_to_go == 0)
	    break;
	}
      rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

      if(i == n_enc)
	{
	  rtdm_event_wait(&ctx->move_done_event);
	}
    } while(i == n_enc);

  rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
  response.motor = i;
  response.result = ctx->enc_info[i].states_to_go;
  if(rtdm_safe_copy_to_user(user_info, buf, &response, sizeof(response)))
    rtdm_printk("TB6612FNG: can't copy data from driver\n");
  ctx->enc_info[i].reported = 1;
  rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
  
  return sizeof(response);
}

/**
 * Write in the device
 *
 * This function is called when the device is written in non-realtime context.
 *
 */
static ssize_t 
pwm_rtdm_write(struct rtdm_dev_context *context,
		   rtdm_user_info_t *user_info,
		   const void *buf, 
		   size_t nbyte)
{
  const ControlCommandRequest *motors = (const ControlCommandRequest*)buf;
  const size_t cnt = nbyte / sizeof(ControlCommandRequest);
  tb6612fng_context_t *ctx = (tb6612fng_context_t*)context->dev_private;  
  rtdm_lockctx_t lock_ctx;
  size_t i;

  if(nbyte == 0)
    return 0;

  if(user_info && !rtdm_rw_user_ok(user_info, buf, nbyte))
    return -EFAULT;

  for(i = 0; i < cnt; ++i)
    {
      if(motors[i].motor >= ARRAY_SIZE(ctx->enc_info))
	continue;
      rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
      ctx->enc_info[motors[i].motor].reported = 1;
      ctx->enc_info[motors[i].motor].states_to_go = motors[i].states_to_go;
      ctx->enc_info[motors[i].motor].target_speed = motors[i].target_speed;
      rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

      setmotordirection(motors[i].motor, motors[i].direction);
    }

  return nbyte;
}

/**
 * Read and write PID configuration and statystic
 *
 */
int 
pwm_rtdm_ioctl(struct rtdm_dev_context *context, 
	       rtdm_user_info_t *user_info, 
	       unsigned int request, 
	       void *arg)
{
  tb6612fng_context_t *ctx = (tb6612fng_context_t*)context->dev_private;  
  rtdm_lockctx_t lock_ctx;
  int err = 0;
  speed_info_t current_speed;
  size_t i;
  pid_config_t pid_configs;

  switch(request)
    {
    case PID_GET_CONFIG:
      for(i = 0; i < ARRAY_SIZE(ctx->pid_params); ++i)
	{
	  pid_configs[i].max = ctx->pid_params[i].max;
	  pid_configs[i].min = ctx->pid_params[i].min;
	  pid_configs[i].Kp = ctx->pid_params[i].K[0];
	  pid_configs[i].Ki = ctx->pid_params[i].K[1];
	  pid_configs[i].Kd = ctx->pid_params[i].K[2];
	}
      if(user_info)
	err = rtdm_safe_copy_to_user(user_info, 
				     arg,
				     &pid_configs,
				     sizeof(pid_config_t));
      else
	memcpy(arg, &pid_configs, sizeof(pid_config_t));
      break;

    case PID_GET_SPEED:
      rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
      for(i = 0; i < ARRAY_SIZE(ctx->enc_info); ++i)
	{
	  current_speed[i].crosstrack_error = ctx->pid_err[i];
	  ctx->pid_err[i] = 0;
	  current_speed[i].direction = ctx->enc_info[i].direction;
	  current_speed[i].delta_t = 
	    ctx->enc_info[i].cur_time - ctx->enc_info[i].prev_time;
	  current_speed[i].state_changes_counter = 
	    ctx->enc_info[i].prev_state_changes_counter;
	}
      rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
      /*
      rtdm_printk("ioctl: %lu  %i\t%lu  %i\n",
		  current_speed[0].delta_t, current_speed[0].direction,
		  current_speed[1].delta_t, current_speed[1].direction);
      */
      if(user_info)
	err = rtdm_safe_copy_to_user(user_info, 
				     arg,
				     &current_speed,
				     sizeof(speed_info_t));
      else
	memcpy(arg, &current_speed, sizeof(speed_info_t));
      break;

    case PID_SET_CONFIG:
      {
	pid_config_t *config;
	pid_config_t config_buf;

	config = (pid_config_t*)arg;

	if(user_info)
	  {
	    err = rtdm_safe_copy_from_user(user_info, 
					   &config_buf,
					   arg,
					   sizeof(pid_config_t));
	    if(err)
	      return err;

	    config = &config_buf;
	  }

	rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
	for(i = 0; i < ARRAY_SIZE(ctx->pid_params); ++i)
	  {
	    ctx->pid_params[i].max = (*config)[i].max;
	    ctx->pid_params[i].min = (*config)[i].min;
	    ctx->pid_params[i].K[0] = (*config)[i].Kp;
	    ctx->pid_params[i].K[1] = (*config)[i].Ki;
	    ctx->pid_params[i].K[2] = (*config)[i].Kd;
	    ctx->pid_params[i].prev_error = 0;
	    ctx->pid_params[i].integral = 0;
	    ctx->pid_err[i] = 0;
	  }
	rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
      }
      break;

    default:
      err = -ENOSYS;
    }

  return err;
}


/**
 * This structure describe the simple RTDM device
 *
 */
static struct rtdm_device device = {
  .struct_version = RTDM_DEVICE_STRUCT_VER,

  .device_flags = RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
  .context_size = sizeof(tb6612fng_context_t),
  .device_name = DEVICE_NAME,

  .open_nrt = pwm_rtdm_open_nrt,

  .ops = {
    .close_nrt = pwm_rtdm_close_nrt,
    //.read_nrt = pwm_rtdm_read_nrt,
    //.write_nrt = pwm_rtdm_write_nrt,
    .read_rt = pwm_rtdm_read,
    .write_rt = pwm_rtdm_write,
    .ioctl_rt = pwm_rtdm_ioctl,
    .ioctl_nrt = pwm_rtdm_ioctl
  },

  .device_class = RTDM_CLASS_EXPERIMENTAL,
  .device_sub_class = RTDM_SUBCLASS_TB6612FNG,
  .profile_version = 1,
  .driver_name = "TB6612FNG-CTL",
  .driver_version = RTDM_DRIVER_VER(0, 1, 2),
  .peripheral_name = "TB6612FNG dual motor driver",
  .provider_name = "Andrey Nechypurenko",
  .proc_name = device.device_name,
};


/**
 * This function is called when the module is loaded
 *
 * It simply registers the RTDM device.
 *
 */
int __init pwm_rtdm_init(void)
{
  int res;

  res = rtdm_dev_register(&device);
  if(res == 0)
    rtdm_printk("TB6612FNG: driver registered without errors\n");
  else
    {
      rtdm_printk("TB6612FNG: driver registration failed: \n");
      switch(res)
	{
	case -EINVAL: 
	  rtdm_printk("The device structure contains invalid entries. "
		      "Check kernel log for further details.");
	  break;

	case -ENOMEM: 
	  rtdm_printk("The context for an exclusive device cannot be allocated.");
	  break;

	case -EEXIST:
	  rtdm_printk("The specified device name of protocol ID is already in use.");
	  break;

	case -EAGAIN: rtdm_printk("Some /proc entry cannot be created.");
	  break;
	
	default:
	  rtdm_printk("Unknown error code returned");
	  break;
	}
      rtdm_printk("\n");
    }

  // Initialize with default values
  res = initpwm();
  if(res != 0)
    rtdm_printk("TB6612FNG: initialization error: %i was returned\n", res);
  else
    {
      setmotorduty(0, 0);
      setmotorduty(1, 0);
    }

  return res;
}

/**
 * This function is called when the module is unloaded
 *
 * It unregister the RTDM device, polling at 1000 ms for pending users.
 *
 */
void __exit pwm_rtdm_exit(void)
{
  rtdm_printk("TB6612FNG: stopping pwm tasks\n");
  cleanuppwm();
  rtdm_dev_unregister(&device, 1000);
  rtdm_printk("TB6612FNG: uninitialized\n");
}


module_init(pwm_rtdm_init);
module_exit(pwm_rtdm_exit);
