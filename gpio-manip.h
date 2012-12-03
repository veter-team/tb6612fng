#ifndef __GPIOMANIP_H
#define __GPIOMANIP_H

#include <linux/types.h>

/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * M0   - Mode 0
 */

#define IEN     (1 << 8)

#define IDIS    (0 << 8)
#define PTU     (1 << 4)
#define PTD     (0 << 4)
#define EN      (1 << 3)
#define DIS     (0 << 3)

#define M0      0
#define M1      1
#define M2      2
#define M3      3
#define M4      4
#define M5      5
#define M6      6
#define M7      7

#define STBY  133
#define PWMA  130
#define AIN1  132
#define AIN2  131
#define PWMB  136
#define BIN1  134
#define BIN2  135
#define ENC1A 143
#define ENC1B 158
#define ENC2A 162
#define ENC2B 161

typedef struct tagChannelConfig
{
  const u32 stby;
  const u32 pwm;
  const u32 in1;
  const u32 in2;
} ChannelConfig;


// Configure and initialize GPIO pins
int InitGPIO(ChannelConfig *channels, size_t ch_count);

// Unmap GPIO memory
void CleanupGPIO(ChannelConfig *channels, size_t ch_count);

#endif // __GPIOMANIP_H
