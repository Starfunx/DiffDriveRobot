#include "differentialDrive.h"


void differential_update(differential_context* diffDrive,
    float linearVelocity, float angularVelocity,
    float* rightWheelVelocity, float* leftWheelVelocity)
{
    linearVelocity = linearVelocity / diffDrive->maxLinearVelocity;
    angularVelocity = angularVelocity / diffDrive->maxAngularVelocity;

    if ((fabs(angularVelocity)+fabs(linearVelocity)) > 1) {
        // set the point in the square maxvelocity maxangularVelocity.
        float m = fabs(angularVelocity)+fabs(linearVelocity);
        angularVelocity = angularVelocity/m;
        linearVelocity = linearVelocity/m;
    }

    linearVelocity = linearVelocity * diffDrive->maxLinearVelocity;
    angularVelocity = angularVelocity * diffDrive->maxAngularVelocity;

    *rightWheelVelocity = (linearVelocity + angularVelocity*diffDrive->distanceBetweenWheels/2);
    *leftWheelVelocity =  (linearVelocity - angularVelocity*diffDrive->distanceBetweenWheels/2);
}
