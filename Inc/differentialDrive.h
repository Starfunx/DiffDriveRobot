#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

void differential(float linearSpeed, float angularSpeed,
    float *wheelSpeedR, float *wheelRadiusL,
    float distanceBetweenWheels);

#endif
