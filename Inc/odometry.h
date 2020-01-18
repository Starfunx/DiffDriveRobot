#ifndef ODOMETRY_H
#define ODOMETRY_H

#ifndef PI
#define PI 3.141592
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include <math.h>
#include "string.h"
#include <stdio.h>

#ifndef POSITION_STRUCT
#define POSITION_STRUCT
typedef struct {
    float x;
    float y;
    float theta;
} _position;
#endif

typedef struct {
    float encoderRes;
    float wheelRadiusR, wheelRadiusL;
    float distanceBetweenWheels;

    _position position;
    float linearDisplacement;
    float angularDisplacement;

    int16_t *rightTicks, *leftTicks;
} odometry_Context;

void odometry_init(odometry_Context *odometry);
void odometry_update(odometry_Context *odometry);
#endif
