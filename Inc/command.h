#ifndef COMMAND_H
#define COMMAND_H

#include "gcode.h"
#include "robot.h"

void peekCommand(gcodeCommand_context* command, diffDriveRobot_Context* robot);


// commands prototypes
void fastMove(diffDriveRobot_Context* robot, float xc, float yc, float thetac);
// void fastMove2(float xc, float yc);
// void directMove(float xc, float yc, float thetac);

void breakeMotors(diffDriveRobot_Context* robot);
void unBreakeMotors(diffDriveRobot_Context* robot);

#endif
