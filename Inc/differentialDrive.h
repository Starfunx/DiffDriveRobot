#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include <stdlib.h>

#ifndef PI
#define PI 3.141592
#endif

typedef struct {
    float distanceBetweenWheels;

    float maxLinearVelocity;
    float maxAngularVelocity;
} differential_context;

void differential_update(differential_context* diffdrive,
    float linearSpeed, float angularSpeed,
    float *wheelSpeedR, float *wheelRadiusL);

#endif
