#ifndef PID_H
#define PID_H

typedef struct {
    float   Kp;
    float   Ti;
    float   Td;
    float   minOut;
    float   maxOut;

    float   integral;
    float   last_error;
} pid_Context;

void pid_init(pid_Context *pid);
float pid_update(pid_Context *pid, float consign, float mesure);

float constrain(float value, float min, float max);

#endif
