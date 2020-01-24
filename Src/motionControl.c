#include "motionControl.h"

void motionControl_init(motionControl_context *motionController){
    motionController->consign.x = 0;
    motionController->consign.y = 0;
    motionController->consign.theta = 0;

    motionController->maxLinearAcceleration = 0;// the user has to set these 4 values!!!!!
    motionController->maxLinearVelocity = 0;
    motionController->maxAngularAcceleration = 0;
    motionController->maxAngularVelocity = 0;
}

void motionControl_update(motionControl_context* motionController, _position robotPos,
                                          float* linearVelocity, float* angularVelocity){
    float alpha = atan2(motionController->consign.y - robotPos.y,motionController->consign.x - robotPos.x) - robotPos.theta;
    float rho  = sqrt(pow(motionController->consign.x - robotPos.x,2) + pow(motionController->consign.y - robotPos.y,2));

    *LinearVelocity = motionController->Krho * rho;
    *angularVelocity = motionController->Kalpha * np.sin(alpha)*np.cos(alpha);

}

void motionControl_setConsign(motionControl_context* motionController,
     float x, float y, float theta){
    motionController->consign.x = x;
    motionController->consign.y = y;
    motionController->consign.theta = theta;
}
