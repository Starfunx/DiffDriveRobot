#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <math.h>
#include "utils.h"

typedef struct{
    float aFrein;
    float aMax;
    float vMax;
    float lastPos;
} ramp_Context;

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

    ramp_Context rampLin;
    ramp_Context rampAng;

    float alpha;
    float rho;

    int movementMode;

    // Deplacement rapide/ linéaire? mode 2^0 = 0/1
    // sens d'arivÃ© important oui/non?
    // sens d'arrivÃ© 1st devant/derierre?
    // angle final oui/non?

    // position d'asservissemnt ? (impact sur les parametres prÃ©cedents??)


} motionControl_Context;


void motionControl_init(motionControl_Context *motionController, float xc, float yc, float thetac);
void motionControl_update(motionControl_Context* motionController, _position robotPos, float* linearVelocity, float* angularVelocity, float dt);
void motionControl_update0(motionControl_Context* motionController, float* linearVelocity, float* angularVelocity, float dt);                       //  fast mode
void motionControl_update1(motionControl_Context* motionController, float* linearVelocity, float* angularVelocity, float dt);                       // linear mode
void motionControl_update2(motionControl_Context* motionController, _position robotPos, float* linearVelocity, float* angularVelocity, float dt);
void motionControl_setConsign(motionControl_Context* motionController,
    float x, float y, float theta);
void motionControl_setConsign(motionControl_Context* motionController,
    float x, float y, float theta);

void alphaRho(_position consignPos, _position robotPos, float* alpha, float* rho);
float ramp_update(ramp_Context* ramp, float pos, float dt);

#endif
