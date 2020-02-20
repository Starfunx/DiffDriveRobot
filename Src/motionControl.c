#include "motionControl.h"

void motionControl_init(motionControl_Context *motionController, float xc, float yc, float thetac){
    motionController->consign.x = xc;
    motionController->consign.y = yc;
    motionController->consign.theta = thetac;
}


void motionControl_update(motionControl_Context* motionController, _position robotPos, float* linearVelocity, float* angularVelocity, float dt){
    alphaRho(motionController->consign, robotPos, &motionController->alpha, &motionController->rho);
    if (motionController->movementMode == 0){
        motionControl_update0(motionController, linearVelocity, angularVelocity, dt);
        // critère d'arrêt
        if (motionController->alpha > -motionController->Salpha && motionController->alpha < motionController->Salpha
            && motionController->rho < motionController->Srho){
            *linearVelocity = 0.;
            *angularVelocity = 0.;
        }
    }
    else if(motionController->movementMode == 1){
        motionControl_update1(motionController, linearVelocity, angularVelocity, dt);
        // critère d'arrêt
        if (motionController->alpha > -motionController->Salpha && motionController->alpha < motionController->Salpha
            && motionController->rho < motionController->Srho){
            *linearVelocity = 0.;
            *angularVelocity = 0.;
        }
    }
    else if(motionController->movementMode == 2){

        // critère d'arrêt
        if ( fabs(motionController->consign.theta - robotPos.theta) < motionController->Salpha){
            *linearVelocity = 0.;
            *angularVelocity = 0.;
        }
        else {
            motionControl_update2(motionController, robotPos, linearVelocity, angularVelocity, dt);
        }
    }

    if (motionController->rho < motionController->Srho){
        motionController->movementMode = 2;
    }

}

void motionControl_update0(motionControl_Context* motionController, float* linearVelocity, float* angularVelocity, float dt){
    *linearVelocity = motionController->Krho * motionController->rho * cos(motionController->alpha);
    // *linearVelocity = ramp_update(&motionController->rampLin, motionController->rho*cos(motionController->alpha), dt);
    *angularVelocity = motionController->Kalpha * sin(motionController->alpha);
    // *angularVelocity = ramp_update(&motionController->rampAng, motionController->alpha*sin(motionController->alpha), dt);

    if (motionController->rho < motionController->Srho){
        *linearVelocity = 0.;
        *angularVelocity = 0.;
    }
}

// motion control with rotations firsts
void motionControl_update1(motionControl_Context* motionController, float* linearVelocity, float* angularVelocity, float dt){
    if (fabs(motionController->alpha) > motionController->Salpha){
        *linearVelocity = 0.;
        *angularVelocity = motionController->Kalpha * motionController->alpha;
    }
    else {
        motionController->movementMode = 0;
    }
}

void motionControl_update2(motionControl_Context* motionController, _position robotPos, float* linearVelocity, float* angularVelocity, float dt){
    if (fabs(motionController->alpha) > motionController->Salpha){
        *linearVelocity = 0.;
        *angularVelocity = motionController->Kalpha * constrainAngle(motionController->consign.theta - robotPos.theta);
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
