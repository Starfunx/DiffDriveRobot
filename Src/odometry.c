#include "odometry.h"

void odometry_init(odometry_Context *odometry, float x0, float y0, float theta0){
    odometry->linearDisplacement =0.;
    odometry->angularDisplacement =0.;

    odometry->position.x     = x0;
    odometry->position.y     = y0;
    odometry->position.theta = constrainAngle(theta0); // remise dans ]-pi pi]
}

void odometry_update(odometry_Context *odometry){
    float distR =  odometry->wheelRadiusR * 2*PI/odometry->encoderRes * (float)(int16_t)*(odometry->rightTicks);
    float distL =  odometry->wheelRadiusL * 2*PI/odometry->encoderRes * (float)(int16_t)*(odometry->leftTicks);
    *(odometry->rightTicks) = 0;
	*(odometry->leftTicks) = 0;

    odometry->linearDisplacement = (distR + distL)/2;
    odometry->angularDisplacement = (distR - distL)/odometry->distanceBetweenWheels;

    odometry->position.x = odometry->position.x + odometry->linearDisplacement*cos(odometry->position.theta + odometry->angularDisplacement/2);
    odometry->position.y = odometry->position.y + odometry->linearDisplacement*sin(odometry->position.theta + odometry->angularDisplacement/2);
    odometry->position.theta = odometry->position.theta + odometry->angularDisplacement;
    odometry->position.theta = constrainAngle(odometry->position.theta);
}
