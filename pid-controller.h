#ifndef __PIDCONTROLLER_H
#define __PIDCONTROLLER_H

#include "pid-params.h"

/**
 * Calculates PID output based on the supplied parameters,
 * target_value and current_value. Output is clamped by params->max
 * and params->min.
 */
int pidcalc(pidparams_t *params, int target_value, int current_value);

#endif // __PIDCONTROLLER_H
