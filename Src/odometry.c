#include "odometry.h"

void odometry_init(odometry_Context *odometry){
    odometry->linearDisplacement =0.;
    odometry->angularDisplacement =0.;
}

void odometry_update(odometry_Context *odometry){
    float distR = (float)(int16_t)*(odometry->encoderR) / odometry->encoderRes * odometry->wheelRadiusR;
    float distL = (float)(int16_t)*(odometry->encoderL) / odometry->encoderRes * odometry->wheelRadiusL;
    odometry->encoderR = 0;
    odometry->encoderL = 0;

    odometry->linearDisplacement = (distR + distL)/2;
    odometry->angularDisplacement = (distR - distL)/2;

    odometry->position.x = odometry->position.x + odometry->angularDisplacement*cos(odometry->position.theta + odometry->angularDisplacement/2);
    odometry->position.y = odometry->position.y + odometry->angularDisplacement*sin(odometry->position.theta + odometry->angularDisplacement/2);
    odometry->position.theta = odometry->position.theta + odometry->angularDisplacement;
}
