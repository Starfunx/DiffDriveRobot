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
} motionControl_Context;

void motionControl_init(motionControl_Context *motionController, float xc, float yc, float thetac);
void motionControl_update(motionControl_Context* motionController, _position robotPos,
    float* LinearVelocity, float* AngularVelocity);
void motionControl_setConsign(motionControl_Context* motionController,
    float x, float y, float theta);

#endif
