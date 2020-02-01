#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <math.h>
#include "utils.h"

typedef struct {
    _position consign;
    float maxLinearAcceleration;
    float maxLinearVelocity;
    float maxAngularAcceleration;
    float maxAngularVelocity;
    float Krho;
    float Kalpha;

    float alpha;
    float rho;
} motionControl_context;

void motionControl_init(motionControl_context *motionController);
void motionControl_update(motionControl_context* motionController, _position robotPos,
    float* LinearVelocity, float* AngularVelocity);
void motionControl_setConsign(motionControl_context* motionController,
    float x, float y, float theta);

#endif
