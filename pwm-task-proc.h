#ifndef __PWM_TASK_PROC_H
#define __PWM_TASK_PROC_H

/**
 * Initialize PWM generation infrastructure.
 *
 * @return 0 on success or error code in case of failure.
 */
int initpwm(void);



/**
 * Turn specified motor on/off by enabling/disabling controllers
 * standby mode.
 *
 * @param motor - specify the motor to turn on/off
 *
 * @param on - specify whether to turn on (1) or off(0)
 */
void turnmotor(int motor, int on);

/**
 * Set the width of the pulse for certain channel.
 *
 * @param channel - specify the output channel to work with.
 *
 * @param percentage - pulse width specified in percents of the
 * maximal allowed width. Should be in range between 0 and 100. The
 * actual width which will be set is implementation specific and may
 * vary depending on devices to be controlled. Current implementation
 * will set the width in range between 600 and 2000 usec which is the
 * typical range for model servos such as for example Futaba servos.
 */
void setmotorduty(int motor, int percentage);

/**
 * Set motor rotation direction
 *
 * @param channel - specify the output channel to work with.
 *
 * @param direction - forward if true backward if false
 */
void setmotordirection(int channel, int direction);

/**
 * Return current PWM width in percents of the maximum width.
 *
 * @param channel - specify the output channel of interest.
 *
 * @return PWM width withing 0 to 100 range
 */
nanosecs_rel_t getmotorduty(int channel);


/**
 * Release acquired resources and stops real-time threads started for
 * PWM generation.
 */
void cleanuppwm(void);

#endif
