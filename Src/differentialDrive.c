#include "differentialDrive.h"

void differential_init(differential_context *differential)
{
    differential->maxWheelPwmValue = 255.;
}

void differential_update(differential_context* diffDrive,
    float linearVelocity, float angularVelocity,
    float* rightWheelVelocity, float* leftWheelVelocity)
{
    linearVelocity = linearVelocity / diffDrive->maxLinearVelocity;
    angularVelocity = angularVelocity / diffDrive->maxAngularVelocity;

    if ((abs(angularVelocity)+abs(linearVelocity)) > 1) {
        // set the point in the square maxvelocity maxangularVelocity.
        float m = linearVelocity + angularVelocity;
        angularVelocity = angularVelocity/m;
        linearVelocity = linearVelocity/m;
    }

    linearVelocity = linearVelocity * diffDrive->maxWheelPwmValue;
    angularVelocity = angularVelocity * diffDrive->maxWheelPwmValue;

    *rightWheelVelocity = (linearVelocity + angularVelocity*diffDrive->distanceBetweenWheels/2)*2*PI/diffDrive->rightWheelRadius
    *leftWheelVelocity =  (linearVelocity - angularVelocity*diffDrive->distanceBetweenWheels/2)*2*PI/diffDrive->leftWheelRadius
}
