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
    float Srho;
    float Salpha;

    float alpha;
    float rho;
} motionControl_Context;

typedef struct{
    float aFrein;
    float aMax;
    float vMax;
    float lastPos;
} ramp_Context;

void motionControl_init(motionControl_Context *motionController, float xc, float yc, float thetac);
void motionControl_update(motionControl_Context* motionController, _position robotPos,
    float* LinearVelocity, float* AngularVelocity);
void motionControl_update2(motionControl_Context* motionController, _position robotPos,
    float* LinearVelocity, float* AngularVelocity);
void motionControl_setConsign(motionControl_Context* motionController,
    float x, float y, float theta);
void motionControl_setConsign(motionControl_Context* motionController,
    float x, float y, float theta);

void alphaRho(_position consignPos, _position robotPos, float* alpha, float* rho);
float ramp_update(ramp_Context* ramp, float pos, float dt);

#endif
