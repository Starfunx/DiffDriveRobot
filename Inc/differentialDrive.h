#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include <stdlib.h>
#include <math.h>

#ifndef PI
#define PI 3.141592
#endif

typedef struct {
    float distanceBetweenWheels;

    float maxLinearVelocity;
    float maxAngularVelocity;
} differential_Context;

void differential_update(differential_Context* diffdrive,
    float linearSpeed, float angularSpeed,
    float *wheelSpeedR, float *wheelRadiusL);

#endif
