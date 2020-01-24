#include "pid.h"

void pid_init(pid_Context *pid){
   pid->integral =     0.;
   pid->last_error =   0.;
   }

float pid_update(pid_Context *pid, float consign, float mesure){
   float error = consign - mesure;
   float proportional = pid->Kp * error;
   pid->integral = pid->integral + pid->Ti * error; //euler integration
   pid->integral = constrain(pid->integral, pid->minOut, pid->maxOut);
   float derivate = pid->Td*(error - pid->last_error);
   pid->last_error = error;
   float retval = proportional + derivate + pid->integral;
   return constrain(retval, pid->minOut, pid->maxOut);
}

float constrain(float value,float min,float max){
   if (value < min){
       return min;
   }
   if (value > max){
       return max;
   }
   return value;
}
