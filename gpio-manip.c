#include "gpio-manip.h"
#include <rtdm/rtdm_driver.h>
#include <linux/gpio.h>


static struct gpio motor_ctl_gpios[] = 
{
  { STBY, GPIOF_OUT_INIT_LOW, "Motor standby" },
  { PWMA, GPIOF_OUT_INIT_LOW, "PWM A" },
  { AIN1, GPIOF_OUT_INIT_LOW, "AIN1"   },
  { AIN2, GPIOF_OUT_INIT_LOW, "AIN2"  },
  { PWMB, GPIOF_OUT_INIT_LOW, "PWM B" },
  { BIN1, GPIOF_OUT_INIT_LOW, "BIN1"   },
  { BIN2, GPIOF_OUT_INIT_LOW, "BIN2"  },
  { ENC1A, GPIOF_IN, "ENC1A"  },
  { ENC1B, GPIOF_IN, "ENC1B"  },
  { ENC2A, GPIOF_IN, "ENC2A"  },
  { ENC2B, GPIOF_IN, "ENC2B"  }
};


static void ConfigInputPin(u32 val,
			   void __iomem *confmem,
			   u16 base_offset)
{
  // GPIO_RISINGDETECT
  iowrite32(val, confmem + base_offset + 0x48);
  //GPIO_FALLINGDETECT
  iowrite32(val, confmem + base_offset + 0x4C);
  // GPIO_SETIRQENABLE1
  iowrite32(val, confmem + base_offset + 0x64);
}


int 
InitGPIO(ChannelConfig *channels, size_t ch_count)
{
  const u16 mode_value = (PTD | M4);
  void __iomem *confmem = NULL;
  int res = 0;

  rtdm_printk("TB6612FNG: configuring I/O pins mode\n");
  confmem = ioremap(0x48000000, 0x05cc);
  if(!confmem)
    {
      rtdm_printk("TB6612FNG: pinconf mapping failed\n");
      return 1;
    }

  // Pinmuxing

  // STBY GPIO 133
  iowrite16(mode_value, confmem + 0x215C + 2);
  // PWMA GPIO 130
  iowrite16(mode_value, confmem + 0x2158);
  // AIN1 GPIO 132
  iowrite16(mode_value, confmem + 0x215C);
  // AIN2 GPIO 131
  iowrite16(mode_value, confmem + 0x2158 + 2);
  // PWMB GPIO 136
  iowrite16(mode_value, confmem + 0x2164);
  // BIN1 GPIO 134
  iowrite16(mode_value, confmem + 0x2160);
  // BIN2 GPIO 135
  iowrite16(mode_value, confmem + 0x2160 + 2);

  // ENC1 GPIO 143
  iowrite16(IEN | mode_value, confmem + 0x2170 + 2);

  // ENC2 GPIO 158
  iowrite16(IEN | mode_value, confmem + 0x2190);

  // ENC3 GPIO 162
  iowrite16(IEN | mode_value, confmem + 0x2198);

  // ENC4 GPIO 161
  iowrite16(IEN | mode_value, confmem + 0x2194+2);

  iounmap(confmem);

  res = gpio_request_array(motor_ctl_gpios, ARRAY_SIZE(motor_ctl_gpios));
  if(res)
    {
      rtdm_printk("TB6612FNG: error requesting GPIO\n");
      return res;
    }

  confmem = ioremap(0x49050000, 0x05cc);
  if(!confmem)
    {
      rtdm_printk("TB6612FNG: GPIO bank mapping failed\n");
      return 2;
    }

  // Enable interrupts for rotary encoders

  // 15 = (gpio)143 - 128
  // 30 = (gpio)158 - 128
  ConfigInputPin((1 << 15) | (1 << 30), confmem, 0x6000);

  // 2 = (gpio)162 - 160
  // 1 = (gpio)161 - 160
  ConfigInputPin((1 << 2) | (1 << 1), confmem, 0x8000);

  iounmap(confmem);

  return res;
}


void 
CleanupGPIO(ChannelConfig *channels, size_t ch_count)
{
  gpio_free_array(motor_ctl_gpios, ARRAY_SIZE(motor_ctl_gpios));
}
