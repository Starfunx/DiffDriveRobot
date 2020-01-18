#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#ifndef PI
#define PI 3.141592
#endif

typedef struct {
    float distanceBetweenWheels;
    float rightWheelRadius;
    float leftWheelRadius;

    float maxLinearVelocity;
    float maxAngularVelocity;
    float maxWheelPwmValue;
} differential_context;

void differential_init(differential_context *differential);
void differential_update(differential_context* diffdrive,
    float linearSpeed, float angularSpeed,
    float *wheelSpeedR, float *wheelRadiusL);

#endif
