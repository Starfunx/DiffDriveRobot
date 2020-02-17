#ifndef COMMAND_H
#define COMMAND_H

#include "stm32l4xx_hal.h" // for HAL UART Transmit
#include "gcode.h"
#include "robot.h"

void peekCommand(gcodeCommand_context* command, diffDriveRobot_Context* robot, UART_HandleTypeDef *huart);


// commands prototypes

//motion control setters
void fastMove(diffDriveRobot_Context* robot, float xc, float yc);
void fastMoveTheta(diffDriveRobot_Context* robot, float xc, float yc, float thetac);
void linearMove(diffDriveRobot_Context* robot, float xc, float yc);
void linearMoveTheta(diffDriveRobot_Context* robot, float xc, float yc, float thetac);

void setRobotSpeeds(diffDriveRobot_Context* robot, float linearSpeed, float AngularSpeed);
void setWheelsSpeeds(diffDriveRobot_Context* robot, float rightWheelSpeed, float leftWheelSpeed);

void breakeMotors(diffDriveRobot_Context* robot);
void unBreakeMotors(diffDriveRobot_Context* robot);

// getters
void printPosition(diffDriveRobot_Context* robot, UART_HandleTypeDef *huart);
void printComTest(UART_HandleTypeDef *huart);
void printSelf(UART_HandleTypeDef *huart);

// values setters
void setPosition(diffDriveRobot_Context* robot, float x, float y, float theta);

void setPID_D(diffDriveRobot_Context* robot, float Kp, float Ti, float Td);
void setPID_G(diffDriveRobot_Context* robot, float Kp, float Ti, float Td);
void setLOdomWheelDia(diffDriveRobot_Context* robot, float wheelDiameter);
void setROdomWheelDia(diffDriveRobot_Context* robot, float wheelDiameter);
void setOdomInnerDist(diffDriveRobot_Context* robot, float innerDist);
void setMotorInnerDist(diffDriveRobot_Context* robot, float innerDist);




#endif
