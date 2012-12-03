#include <rtdm/rtdm_driver.h>
#include <linux/gpio.h>
#include "enc-irq-handling.h"
#include "gpio-manip.h"
#include "pwm-task-proc.h"


// Quadrature Encoder Matrix
static const char QEM[16] = 
{ // curr value
   0,-1, 1, 2, // p
   1, 0, 2,-1, // r
  -1, 2, 0, 1, // e
   2, 1,-1, 0  // v
};

rtdm_irq_t irq_handles[N_ENCODERS];
EncoderInfo enc_info[N_ENCODERS];
extern rtdm_event_t move_done_event;

int 
encoder_irq_handler(rtdm_irq_t *irq_context)
{
  tb6612fng_context *ctx;

  ctx = rtdm_irq_get_arg(irq_context, tb6612fng_context_t);

  for(i = 0; i < sizeof(ctx->irq_handles)/sizeof(ctx->irq_handles[0]); ++i)
    if(irq_handle == &ctx->irq_handles[i])
      break;

  rtdm_lock_get(&ctx->lock);

  ctx->enc_info[i].prev_time = ctx->enc_info[i].cur_time;
  ctx->enc_info[i].cur_time = rtdm_clock_read_monotonic();

  ctx->enc_info[i].prev_value = ctx->enc_info[i].cur_value;
  // New = inputA * 2 + inputB
  ctx->enc_info[i].cur_value = 0;// <<< Here we would actually read values of the two encoder pins (inputA and inputB)
  ctx->enc_info[i].direction = 
    QEM[ctx->enc_info[i].prev_value * 4 + ctx->enc_info[i].cur_value];

  if(ctx->enc_info[i].states_to_go == 0)
      return RTDM_IRQ_HANDLED;

  if(ctx->enc_info[i].states_to_go == 1)
    {
      ctx->enc_info[i].states_to_go = 0;
      ctx->enc_info[i].reported = 0;
      setmotorduty(i, 0);
      //rtdm_event_pulse(&move_done_event);
      rtdm_event_signal(&ctx->move_done_event);
    }
  else
    --ctx->enc_info[i].states_to_go;

  rtdm_lock_put(&ctx->lock);

  return RTDM_IRQ_HANDLED;
}


int 
InitializeEncoders(void)
{
  int res = 0;
  size_t i;

  for(i = 0; i < N_ENCODERS; ++i)
    {
      enc_info[i].cur_value = 0;
      enc_info[i].prev_value = 0;
      enc_info[i].direction = 0;
      enc_info[i].target_speed = 0;
      enc_info[i].prev_time = 0;
      enc_info[i].cur_time = 0;
      enc_info[i].states_to_go = 0;
      enc_info[i].reported = 1;
    }

  res = rtdm_irq_request(&irq_handles[0], 
			 gpio_to_irq(ENC1A), // 303
			 encoderIrqHandler, 
			 RTDM_IRQTYPE_EDGE, 
			 "wheel-encoder1", 
			 NULL);
  if(res)
    rtdm_printk("TB6612FNG: error requesting encoder interrupt: %i\n",
		res);

  return res;
}


int 
CleanupEncoders(void)
{
  int res = rtdm_irq_free(&irq_handles[0]);
  return res;
}
