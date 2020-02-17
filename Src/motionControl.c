#include "motionControl.h"

void motionControl_init(motionControl_Context *motionController, float xc, float yc, float thetac){
    motionController->consign.x = xc;
    motionController->consign.y = yc;
    motionController->consign.theta = thetac;
}

void motionControl_update(motionControl_Context* motionController, _position robotPos, float* linearVelocity, float* angularVelocity){

    alphaRho(motionController->consign, robotPos, &motionController->alpha, &motionController->rho);

    *linearVelocity = motionController->Krho * motionController->rho * cos(motionController->alpha);
    *angularVelocity = motionController->Kalpha * sin(motionController->alpha);

    if (motionController->rho < motionController->Srho){
        *angularVelocity = motionController->Kalpha * (motionController->consign.theta - robotPos.theta);
    }

    // critère d'arrêt
    if (fabs(motionController->alpha) < motionController->Salpha &&
        motionController->rho < motionController->Srho){
        *linearVelocity = 0.;
        *angularVelocity = 0.;
    }
}

// motion control with rotations firsts
void motionControl_update2(motionControl_Context* motionController, _position robotPos, float* linearVelocity, float* angularVelocity){

    alphaRho(motionController->consign, robotPos, &motionController->alpha, &motionController->rho);

    *linearVelocity = motionController->Krho * motionController->rho * cos(motionController->alpha);
    *angularVelocity = motionController->Kalpha * motionController->alpha;
    if (motionController->alpha > motionController->Salpha){
        *linearVelocity = 0.;
        *angularVelocity = motionController->Kalpha * motionController->alpha ;
    }

    // critère d'arrêt
    if (fabs(motionController->alpha) < motionController->Salpha &&
        motionController->rho < motionController->Srho){
        *linearVelocity = 0.;
        *angularVelocity = 0.;
    }

}

void motionControl_setConsign(motionControl_Context* motionController,
    float xc, float yc, float thetac){
    motionController->consign.x = xc;
    motionController->consign.y = yc;
    motionController->consign.theta = thetac;
}


void alphaRho(_position consign, _position robotPos, float* alpha, float* rho){
   *alpha = constrainAngle( atan2(consign.y - robotPos.y,consign.x - robotPos.x) - robotPos.theta );
   *rho  = sqrt(pow(consign.x - robotPos.x,2) + pow(consign.y - robotPos.y,2));
}

float ramp_update(ramp_Context* ramp, float pos, float dt){

   float speed = pos - ramp->lastPos;
   float dFrein = pow(speed,2)/(2*ramp->aFrein);
   if (pos < dFrein){
       speed = speed - ramp->aFrein*dt;
   }
   else {
       if (speed < ramp->vMax) {
           speed = speed + ramp->aMax*dt;
       }
       else {
           speed = ramp->vMax;
       }
   }
   return speed;
}
