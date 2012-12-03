#include <stdio.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sched.h>
#include <rtdm/rtdm.h>
#include <native/task.h>
#include "../interface.h"
#include "../pid-params.h"


void 
testPIDParams(int device, unsigned char motor, pid_instance_params_t *pid_params)
{
  ssize_t size;
  float target_speed;
  float step = (MAX_SPEED-MIN_SPEED) / 5;
  size_t i;
  size_t n = 0;
  speed_info_t current_speed;
  ControlCommandResponse read;
  pid_config_t pid_cfg;
  ControlCommandRequest cmd = 
    {
      .motor = motor,
      .direction = 0
    };

  FILE *tspeed = fopen("target_speed.dat", "w");
  FILE *ospeed = fopen("observed_speed.dat", "w");

  for(i = 0; i < sizeof(pid_cfg) / sizeof(pid_cfg[0]); ++i)
    {
      pid_cfg[i].max = (*pid_params)[i].max;
      pid_cfg[i].min = (*pid_params)[i].min;
      pid_cfg[i].Kp = (*pid_params)[i].K[0];
      pid_cfg[i].Ki = (*pid_params)[i].K[1];
      pid_cfg[i].Kd = (*pid_params)[i].K[2];
    }
  // Set PID parameters for current run
  rt_dev_ioctl(device, PID_SET_CONFIG, &pid_cfg);

  // Increasing speed step-wise
  for(target_speed = MIN_SPEED; target_speed < MAX_SPEED; target_speed += step)
    {
      cmd.target_speed = SPEED_TO_STATES(target_speed);
      cmd.states_to_go = 333 * 10; // 10 revolutions
      size = rt_dev_write(device, (const void*)&cmd, sizeof(cmd));
      for(i = 0; i < 100; ++i)
	{
	  rt_dev_ioctl(device, PID_GET_SPEED, &current_speed);
	  fprintf(tspeed, "%u\t%i\n", 
		  n,
		  cmd.target_speed);
	  fprintf(ospeed, "%u\t%i\n", 
		  n,
		  current_speed[cmd.motor].state_changes_counter);
	  ++n;
	  rt_task_sleep(DELTA_T * 1000000);
	}
      size = rt_dev_read(device, (void*)&read, sizeof(read));
    }

  // Decreasing speed
  for(target_speed -= step; target_speed > MIN_SPEED; target_speed -= step)
    {
      cmd.target_speed = SPEED_TO_STATES(target_speed);
      cmd.states_to_go = 333 * 10; // 10 revolutions
      size = rt_dev_write(device, (const void*)&cmd, sizeof(cmd));
      for(i = 0; i < 100; ++i)
	{
	  rt_dev_ioctl(device, PID_GET_SPEED, &current_speed);
	  fprintf(tspeed, "%u\t%i\n", 
		  n,
		  cmd.target_speed);
	  fprintf(ospeed, "%u\t%i\n", 
		  n,
		  current_speed[cmd.motor].state_changes_counter);
	  ++n;
	  rt_task_sleep(DELTA_T * 1000000);
	}
      size = rt_dev_read(device, (void*)&read, sizeof(read));
    }

  fclose(tspeed);
  fclose(ospeed);

  cmd.target_speed = 0;
  rt_dev_write(device, (const void*)&cmd, sizeof(cmd));
}


long unsigned int 
run(int device, unsigned char motor, pid_instance_params_t *pid_params)
{
  ssize_t size;
  float target_speed;
  float step = (MAX_SPEED-MIN_SPEED) / 5;
  const size_t n_iterations2 = (MAX_SPEED - MIN_SPEED) / step / 2;
  size_t i;
  size_t n = 0;
  long unsigned int err = 0;
  ControlCommandRequest cmd = 
    {
      .motor = motor,
      .direction = 0
    };
  speed_info_t current_speed;

  ControlCommandResponse read;

  pid_config_t pid_cfg;
  for(i = 0; i < sizeof(pid_cfg) / sizeof(pid_cfg[0]); ++i)
    {
      pid_cfg[i].max = (*pid_params)[i].max;
      pid_cfg[i].min = (*pid_params)[i].min;
      pid_cfg[i].Kp = (*pid_params)[i].K[0];
      pid_cfg[i].Ki = (*pid_params)[i].K[1];
      pid_cfg[i].Kd = (*pid_params)[i].K[2];
    }

  // Set PID parameters for current run
  rt_dev_ioctl(device, PID_SET_CONFIG, &pid_cfg);

  // To clean up square error
  rt_dev_ioctl(device, PID_GET_SPEED, &current_speed);
  
  for(target_speed = MIN_SPEED+step; target_speed < MAX_SPEED; target_speed += step)
    {
      cmd.target_speed = SPEED_TO_STATES(target_speed);
      cmd.states_to_go = 333 * 2; // 2 revolutions
      size = rt_dev_write(device, (const void*)&cmd, sizeof(cmd));
      size = rt_dev_read(device, (void*)&read, sizeof(read));
      if(n >= n_iterations2)
	{
	  rt_dev_ioctl(device, PID_GET_SPEED, &current_speed);
	  err += current_speed[cmd.motor].crosstrack_error;
	}
      ++n;
    }

  cmd.target_speed = 0;
  rt_dev_write(device, (const void*)&cmd, sizeof(cmd));
  usleep(500*1000);

  return err;
}


int 
sum(int *array, size_t len)
{
  int sum = 0;
  size_t i = 0;
  for(; i < len; ++i)
    sum += array[i];
  return sum;
}


long unsigned int 
twiddleMotor(int device, unsigned char motor, 
	     pid_instance_params_t *pid_params, int tolerance)
{
  int dparams[3] = {1000, 1000, 1000};
  const size_t param_len = sizeof(dparams) / sizeof(dparams[0]);
  size_t i;
  long unsigned int err;
  size_t n = 0;
  int best_error;
  printf("\ttrying:[%i %i %i]\n", (*pid_params)[motor].K[0], (*pid_params)[motor].K[1], (*pid_params)[motor].K[2]);
  best_error = run(device, motor, pid_params);
  printf("\t\tinitial best error: %lu\n", best_error);

  while(sum(dparams, param_len) > tolerance)
    {
      for(i = 0; i < param_len; ++i)
	{
	  (*pid_params)[motor].K[i] += dparams[i];
	  printf("\ttrying:[%i %i %i]\n", (*pid_params)[motor].K[0], (*pid_params)[motor].K[1], (*pid_params)[motor].K[2]);
	  err = run(device, motor, pid_params);
	  printf("\t\tsquare error: %lu\n", err);
	  if(err < best_error)
	    {
	      printf("\t got smaller error\n");
	      best_error = err;
	      dparams[i] *= 1.1;
	    }
	  else
	    {
	      (*pid_params)[motor].K[i] -= 2 * dparams[i];
	      printf("\ttrying:[%i %i %i]\n", (*pid_params)[motor].K[0], (*pid_params)[motor].K[1], (*pid_params)[motor].K[2]);
	      err = run(device, motor, pid_params);
	      printf("\t\tsquare error: %lu\n", err);
	      if(err < best_error)
		{
		  printf("\t got smaller error\n");
		  best_error = err;
		  dparams[i] *= 1.1;
		}
	      else
		{
		  (*pid_params)[motor].K[i] += dparams[i];
		  dparams[i] *= 0.9;
		}
	    }
	}
      n += 1;
      /*
      printf("Twiddle #%u\tparams:[%i %i %i]\tdparams:[%i %i %i]\tBest error: %i\n",
	     n,
	     pid_params->K[0], pid_params->K[1], pid_params->K[2],
	     dparams[0], dparams[1], dparams[2],
	     best_error);
      */
    }
  return best_error;

}


void motor(void *arg)
{
  int device = (int)arg;

  pid_instance_params_t pid_params = 
    {
      {.max = 10, .min = -10, .K = {0, 0, 0}},
      {.max = 10, .min = -10, .K = {0, 0, 0}}
    };
  /*
  long unsigned int best_error = twiddleMotor(device, 0, &pid_params, 1);
  printf("Tuned PID params 0:[%i %i %i]\tBest error: %lu\n",
	 pid_params[0].K[0], pid_params[0].K[1], pid_params[0].K[2],
	 best_error);

  best_error = twiddleMotor(device, 1, &pid_params, 1);
  printf("Tuned PID params 1:[%i %i %i]\tBest error: %i\n",
	 pid_params[1].K[0], pid_params[1].K[1], pid_params[1].K[2],
	 best_error);
  */
  pid_params[1].max = 10;
  pid_params[1].min = -pid_params[1].max;
  pid_params[1].K[0] = 2380;
  pid_params[1].K[1] = 0;
  pid_params[1].K[2] = 1736;
  printf("\n * Test run *\n");
  printf("Two files, target_speed.dat and observed_speed.dat\n");
  printf("will be created for PID performance analysis.\n");
  testPIDParams(device, 1, &pid_params);
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
      return 3;
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
