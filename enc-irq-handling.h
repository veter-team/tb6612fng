#ifndef __ENCIRQHANDLING_H
#define __ENCIRQHANDLING_H

#define N_ENCODERS 2

typedef struct tagEncoderInfo
{
  u8 prev_value; // gpio values for direction detection
  u8 cur_value;
  nanosecs_rel_t target_speed;
  nanosecs_abs_t prev_time;
  char direction;
  nanosecs_abs_t cur_time;
  volatile u32 states_to_go;
  volatile u8 reported;
} EncoderInfo;

int InitializeEncoders(void);
int CleanupEncoders(void);
#endif // __ENCIRQHANDLING_H
