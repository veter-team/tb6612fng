#include <stdio.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sched.h>
#include <rtdm/rtdm.h>
#include <native/task.h>
#include "../interface.h"


void 
testMotor(int device, unsigned char motor)
{
  ssize_t size;
  float c;
  ControlCommandRequest cmd = 
    {
      .motor = motor,
      .direction = 0
    };

  ControlCommandResponse read;

  printf("Testing motor %i\n", (int)motor);

  for(c = MIN_SPEED; c <= MAX_SPEED; c += (MAX_SPEED-MIN_SPEED)/10)
    {
      cmd.target_speed = SPEED_TO_STATES(c);
      cmd.states_to_go = 333 * 1; // 1 revolutions
      size = rt_dev_write(device, (const void*)&cmd, sizeof(cmd));
      printf("Write to device %s#%i: %d bytes. Duty: %i\n", 
	     DEVICE_NAME, motor, size, (int)(cmd.target_speed));

      size = rt_dev_read(device, (void*)&read, sizeof(read));
      printf("Read from device %s: %d bytes - motor[%i] state %i\n", 
	     DEVICE_NAME, 
	     size, 
	     read.motor,
	     read.result);

      sleep(1);
    }

  cmd.target_speed = 0;
  rt_dev_write(device, (const void*)&cmd, sizeof(cmd));
}


void 
testTwoMotors(int device)
{
  ssize_t size;
  float c;
  ControlCommandRequest cmd[2] = 
    {
      {.motor = 0, .direction = 0},
      {.motor = 1, .direction = 0}
    };

  ControlCommandResponse read;

  printf("Testing two motors simultaneously\n");

  for(c = MIN_SPEED; c <= MAX_SPEED; c += (MAX_SPEED-MIN_SPEED)/10)
    {
      cmd[0].target_speed = SPEED_TO_STATES(c);
      cmd[1].target_speed = SPEED_TO_STATES(c);
      cmd[0].states_to_go = 333 * 1; // 1 revolutions
      cmd[1].states_to_go = 333 * 1; // 1 revolutions
      size = rt_dev_write(device, (const void*)cmd, sizeof(cmd));
      printf("Write to device %s: %d bytes. Duty: %i\n", 
	     DEVICE_NAME, size, (int)(cmd[0].target_speed));

      size = rt_dev_read(device, (void*)&read, sizeof(read));
      printf("Read from device %s: %d bytes - motor[%i] state %i\n", 
	     DEVICE_NAME, 
	     size, 
	     read.motor,
	     read.result);

      sleep(1);
    }

  cmd[0].target_speed = 0;
  cmd[1].target_speed = 0;
  rt_dev_write(device, (const void*)cmd, sizeof(cmd));
}


void motor(void *arg)
{
  int device = (int)arg;

  testMotor(device, 0);
  testMotor(device, 1);
  testTwoMotors(device);

}


int 
main(int argc, char *argv)
{
  unsigned char c;
  int device;
  int ret;
  struct rlimit resource_limit;
  RT_TASK motor_task;

  // Avoids memory swapping for this program

  if(0 != mlockall(MCL_CURRENT | MCL_FUTURE))
    {
      perror("mlockall() failed: ");
      fprintf(stderr, "errno = %d.\n", errno);      
      return 1;
    }

  // Set resource limits to allow us to set scheduler
  /*
  printf("User id: %i\n", getuid());
  getrlimit(RLIMIT_RTPRIO, &resource_limit);
  printf("Previous limits: soft=%lld; hard=%lld\n",
	 (long long) resource_limit.rlim_cur, (long long) resource_limit.rlim_max);
  printf("Max FIFO priority is: %i .\n Setting max limit to 99\n", sched_get_priority_max(SCHED_FIFO));
  resource_limit.rlim_cur = 99;
  resource_limit.rlim_max = 99;
  if(0 != setrlimit(RLIMIT_RTPRIO, &resource_limit))
    {
      perror("setrlimit() failed");
      return 1;
    }
  */
  // open the device

  device = rt_dev_open(DEVICE_NAME, 0);
  if(device < 0)
    {
      fprintf(stderr, "ERROR : can't open device %s (%s)\n",
	      DEVICE_NAME, strerror(-device));
      return 2;
    }

  ret = rt_task_create(&motor_task, "motor", 0, 99, T_JOINABLE);
  if(0 != ret)
    {
      fprintf(stderr, "rt_task_create() failed.\n");
      switch(ret)
	{
	case -ENOMEM: 
	  fprintf(stderr, 
		  "ENOMEM: the system fails to get enough dynamic memory from "
		  "the global real-time heap in order to create or register "
		  "the task.\n");
	  return 3;

	case -EEXIST: 
	  fprintf(stderr, 
		  "EEXIST: the name is already in use by some "
		  "registered object.\n");
	  return 3;

	case -EPERM: 
	  fprintf(stderr, 
		  "EPERM: this service was called from an "
		  "asynchronous context.\n");
	  return 3;

	default:
	  fprintf(stderr, "Unknown error code\n");
	  return 3;
	}
    }
  rt_task_start(&motor_task, &motor, (void*)device);

  sleep(1); // to let RT task start
  rt_task_join(&motor_task);

  rt_task_delete(&motor_task);

  // close the device
  ret = rt_dev_close(device);
  if(ret < 0)
    {
      printf("ERROR : can't close device %s (%s)\n",
	     DEVICE_NAME, strerror(-ret));
      return 4;
    }

  return 0;
}
