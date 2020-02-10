#include "motionControl.h"

void motionControl_init(motionControl_Context *motionController, float xc, float yc, float thetac){
    motionController->consign.x = xc;
    motionController->consign.y = yc;
    motionController->consign.theta = thetac;
}

void motionControl_update(motionControl_Context* motionController, _position robotPos,
                                          float* linearVelocity, float* angularVelocity){
    motionController->alpha = atan2(motionController->consign.y - robotPos.y,motionController->consign.x - robotPos.x);
    motionController->alpha = constrainAngle(motionController->alpha) - robotPos.theta;
    motionController->alpha = constrainAngle(motionController->alpha);

    motionController->rho  = sqrt(pow(motionController->consign.x - robotPos.x,2) + pow(motionController->consign.y - robotPos.y,2));

    *linearVelocity = motionController->Krho * motionController->rho * cos(motionController->alpha);
    *angularVelocity = motionController->Kalpha * sin(motionController->alpha);

}

void motionControl_setConsign(motionControl_Context* motionController,
    float xc, float yc, float thetac){
    motionController->consign.x = xc;
    motionController->consign.y = yc;
    motionController->consign.theta = thetac;
}
