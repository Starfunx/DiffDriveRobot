#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include <math.h>

typedef struct {
    float x;
    float y;
    float theta;
} _position;

typedef struct {
    int *encoderR, *encoderL;
    float encoderRes;
    float wheelRadiusR, wheelRadiusL;
    float distanceBetweenWheels;

    _position position;
    float linearDisplacement;
    float angularDisplacement;
} odometry_Context;

void odometry_init(odometry_Context *odometry);
void odometry_update(odometry_Context *odometry);
#endif
