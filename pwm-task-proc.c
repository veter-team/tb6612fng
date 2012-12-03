#include <rtdm/rtdm_driver.h>
#include <linux/gpio.h>
#include "pwm-task-proc.h"
#include "divconst.h"
#include "gpio-manip.h"
#include "enc-irq-handling.h"

#define RANGE_MAP100(c, d, x) (c + ldiv100((d - c) * x))

// 1000us period
#define PERIOD 1000000

// ~ 5% of X
// X / 100 * 5 ~ X / 128 * 4
// = X / 32 = X >> 5
#define DELTA (PERIOD >> 7)

#define RC_NUM 2

ChannelConfig channels[] = 
{
  // Motor A (left)
  {
    .stby = STBY,
    .pwm = PWMA,
    .in1 = AIN1,
    .in2 = AIN2
  },
  // Motor B (right)
  {
    .stby = STBY,
    .pwm = PWMB,
    .in1 = BIN1,
    .in2 = BIN2
  }
};

static rtdm_timer_t up_timer;
static rtdm_timer_t down_timer[RC_NUM];
static nanosecs_rel_t up_interval[RC_NUM];
static uint8_t reconfigured[RC_NUM];
static nanosecs_abs_t down_time[RC_NUM];


// Initialized with default pulse ranges
static int ranges[RC_NUM][2] = {
  {0 + DELTA, PERIOD},
  {0 + DELTA, PERIOD}
};


void 
pwm_up(rtdm_timer_t *timer)
{
  int retval;
  size_t channel = 0;

  for(; channel < RC_NUM; ++channel)
    {
      // set pwm to high
      gpio_set_value(channels[channel].pwm, 1);

      // ideal time to put signal down
      down_time[channel] = rtdm_clock_read_monotonic() + up_interval[channel];
      if(reconfigured[channel])
	{
	  reconfigured[channel] = 0;
	  rtdm_timer_stop(&down_timer[channel]);
	  // request timer to fire DELTA ns earlier then needed
	  retval = rtdm_timer_start(&down_timer[channel], 
				    up_interval[channel] - DELTA,
				    PERIOD,
				    RTDM_TIMERMODE_RELATIVE);
	  if(retval)
	    rtdm_printk("TB6612FNG: error reconfiguring down-timer #%i: %i\n", 
			channel, retval);
	}
    }
}


void 
pwm_down(rtdm_timer_t *timer)
{
  size_t channel = 0;

  // Search for our timer to determine the channel
  for(; channel < sizeof(down_timer) / sizeof(down_timer[0]); ++channel)
    {
      if(timer == &down_timer[channel])
	break;
    }

  // spin until the ideal time to put signal down
  while(down_time[channel] > rtdm_clock_read_monotonic());

  // set pwm to low
  gpio_set_value(channels[channel].pwm, 0);
}


void 
turnmotor(int motor, int on)
{
  gpio_set_value(channels[motor].stby, on);
}


void 
setmotorduty(int channel, int percentage)
{
  if(channel >= RC_NUM)
    channel = RC_NUM - 1;

  //rtdm_printk("TB6612FNG: setmotorduty() %i -> %i\n", channel, percentage);

  if(percentage == 0)
    { // make suere that wheels are not rotating
      // set in1 low
      gpio_set_value(channels[channel].in1, 0);
      // set in2 low
      gpio_set_value(channels[channel].in2, 0);

      up_interval[channel] = ranges[channel][0];

      // enable standby
      //gpio_set_value(channels[channel].stby, 0);
    }
  else
    {
      up_interval[channel] = RANGE_MAP100(ranges[channel][0], 
					  ranges[channel][1], 
					  percentage);
      // disable standby
      gpio_set_value(channels[channel].stby, 1);
    }

  reconfigured[channel] = 1;
}

void 
setmotordirection(int channel, int direction)
{
  if(direction)
    {
      // set in1 low
      gpio_set_value(channels[channel].in1, 0);
      // set in2 high
      gpio_set_value(channels[channel].in2, 1);
    }
  else
    {
      // set in1 high
      gpio_set_value(channels[channel].in1, 1);
      // set in2 low
      gpio_set_value(channels[channel].in2, 0);
    }
}

nanosecs_rel_t 
getmotorduty(int channel)
{
  return up_interval[channel];
}


int 
initpwm(void)
{
  int i;
  int retval;

  for(i = 0; i < RC_NUM; i++)
    {
      up_interval[i] = RANGE_MAP100(ranges[i][0], ranges[i][1], 0);
      reconfigured[i] = 0;
    }

  retval = InitGPIO(channels, sizeof(channels) / sizeof(channels[0]));
  if(retval)
  {
    rtdm_printk("TB6612FNG: GPIO initialization failed\n");
    return retval;
  }
  rtdm_printk("TB6612FNG: GPIO initialized\n");

  rtdm_printk("TB6612FNG: Starting PWM generation timers.\n");
  retval = rtdm_timer_init(&up_timer, pwm_up, "up timer");
  if(retval)
    {
      rtdm_printk("TB6612FNG: error initializing up-timer: %i\n", retval);
      return retval;
    }

  for(i = 0; i < RC_NUM; i++)
    {
      retval = rtdm_timer_init(&down_timer[i], pwm_down, "down timer");
      if(retval)
	{
	  rtdm_printk("TB6612FNG: error initializing down-timer #%i: %i\n", i, retval);
	  return retval;
	}
    }

  retval = rtdm_timer_start(&up_timer, 
			    PERIOD, // we will use periodic timer
			    PERIOD, // PERIOD period
			    RTDM_TIMERMODE_RELATIVE);
  if(retval)
    {
      rtdm_printk("TB6612FNG: error starting up-timer: %i\n", retval);
      return retval;
    }
  rtdm_printk("TB6612FNG: timers created\n");

  return 0;
}


void 
cleanuppwm(void)
{
  int i;

  // enable standby
  gpio_set_value(channels[0].stby, 0);

  rtdm_timer_destroy(&up_timer);
  for(i = 0; i < RC_NUM; ++i)
    rtdm_timer_destroy(&down_timer[i]);

  CleanupGPIO(channels, sizeof(channels) / sizeof(channels[0]));
}
