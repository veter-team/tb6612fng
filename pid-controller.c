#include "pid-controller.h"
#include "divconst.h"


int 
pidcalc(pidparams_t *params, int target_value, int current_value)
{
  int error;
  int derivative;
  int output;

  // Caculate P, I and D errors
  error = target_value - current_value;
  params->integral += error;
  derivative = error - params->prev_error;
  output = divs1000(params->K[0]*error 
		    + params->K[1]*params->integral 
		    + params->K[2]*derivative);

  // Saturation Filter
  if(output > params->max)
    output = params->max;
  else if(output < params->min)
    output = params->min;

  // Update error
  params->prev_error = error;

  return output;
}
