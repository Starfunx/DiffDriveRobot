#ifndef COMMAND_H
#define COMMAND_H

#include "stm32l4xx_hal.h" // for HAL UART Transmit
#include "gcode.h"
#include "robot.h"

void peekCommand(gcodeCommand_context* command, diffDriveRobot_Context* robot, UART_HandleTypeDef *huart);


// commands prototypes
void fastMove(diffDriveRobot_Context* robot, float xc, float yc);
void fastMoveTheta(diffDriveRobot_Context* robot, float xc, float yc, float thetac);
void LinearMove(diffDriveRobot_Context* robot, float xc, float yc);
void LinearMoveTheta(diffDriveRobot_Context* robot, float xc, float yc, float thetac);

void breakeMotors(diffDriveRobot_Context* robot);
void unBreakeMotors(diffDriveRobot_Context* robot);

void printPosition(diffDriveRobot_Context* robot, UART_HandleTypeDef *huart);
void printComTest(UART_HandleTypeDef *huart);
void setPosition(diffDriveRobot_Context* robot, float x, float y, float theta);

void setPID_D(diffDriveRobot_Context* robot, float Kp, float Ti, float Td);
void setPID_G(diffDriveRobot_Context* robot, float Kp, float Ti, float Td);

void printSelf(UART_HandleTypeDef *huart);


#endif
