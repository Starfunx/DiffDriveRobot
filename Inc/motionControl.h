#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <math.h>

#ifndef PI
#define PI 3.141592
#endif

#ifndef POSITION_STRUCT
#define POSITION_STRUCT
typedef struct {
    float x;
    float y;
    float theta;
} _position;
#endif

typedef struct {
    _position consign;
    float maxLinearAcceleration;
    float maxLinearVelocity;
    float maxAngularAcceleration;
    float maxAngularVelocity;
} motionControl_context;

void motionControl_init(motionControl_context *motionController);
void motionControl_update(motionControl_context* motionController, _position robotPos);
void motionControl_setConsign(motionControl_context* motionController,
    float x, float y, float theta);

#endif
