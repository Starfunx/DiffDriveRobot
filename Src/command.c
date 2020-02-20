#include "command.h"


void peekCommand(gcodeCommand_context* command, diffDriveRobot_Context* robot, UART_HandleTypeDef *huart){
    if (hasM(command)) {
        float Kp,Ti,Td, d;
        switch (command->M)
        {
            case 18:
            	breakeMotors(robot);
              	break;
            case 19:
            	unBreakeMotors(robot);
              	break;
            case 114:
                printPosition(robot, huart);
                break;
            case 115:
                printRspeed(robot, huart);
                break;
            case 116:
                printLspeed(robot, huart);
                break;
            case 301:
                ;
                if (hasP(command)) Kp = command->P;
                else Kp = 0.;
                if (hasI(command)) Ti = command->I;
                else Ti = 0.;
                if (hasD(command)) Td = command->D;
                else Td = 0.;
                setPID_D(robot, Kp, Ti, Td);
                break;
            case 302:
                ;
                if (hasP(command)) Kp = command->P;
                else Kp = 0.;
                if (hasI(command)) Ti = command->I;
                else Ti = 0.;
                if (hasD(command)) Td = command->D;
                else Td = 0.;
                setPID_G(robot, Kp, Ti, Td);
                break;
            case 311:
                if (hasD(command)){
                     d = command->D;
                     setROdomWheelDia(robot, d);
                 }
            break;
            case 312:
                if (hasD(command)){
                     d = command->D;
                     setLOdomWheelDia(robot, d);
                 }
            break;
            case 313:
                if (hasD(command)){
                     d = command->D;
                     setOdomInnerDist(robot, d);
                 }
            break;
            case 314:
                if (hasD(command)){
                     d = command->D;
                     setMotorInnerDist(robot, d);
                 }
            break;
            case 321:
                if (hasD(command)){
                     d = command->D;
                     robot->motionController->Srho = d;
                 }
            break;
            case 322:
                if (hasD(command)){
                     d = command->D;
                     robot->motionController->Salpha = d;
                 }
            break;
            case 323:
                if (hasD(command)){
                     d = command->D;
                     robot->motionController->Krho = d;
                 }
            break;
            case 324:
                if (hasD(command)){
                     d = command->D;
                     robot->motionController->Kalpha = d;
                 }
            break;
            case 331: // set max linear speed
                if (hasD(command)){
                     d = command->D;
                     robot->differential->maxLinearVelocity     = d;
                     robot->motionController->rampLin.vMax      = d;
                 }
            break;
            case 332: // set max angular speed
                if (hasD(command)){
                     d = command->D;
                     robot->differential->maxAngularVelocity            = d;
                     robot->motionController->rampAng.vMax              = d;
                 }
            break;
            case 333: // set max linear acceleration
                if (hasD(command)){
                     d = command->D;
                     robot->motionController->rampLin.aMax = d;
                 }
            break;
            case 334: // set max angular acceleration
                if (hasD(command)){
                     d = command->D;
                     robot->motionController->rampAng.aMax = d;
                 }
            break;
            case 335:
                if (hasD(command)){
                     d = command->D;
                     robot->motionController->rampLin.aFrein = d;
                 }
            break;
            case 336:
                if (hasD(command)){
                     d = command->D;
                     robot->motionController->rampAng.aFrein = d;
                 }
            break;
            case 666:
                printComTest(huart);
                break;
            case 667:
                printSelf(huart);
                break;
            default:
            	break;// default statements
        }
    }
    if (hasN(command)) {
        switch (command->N)
        {
            default:
            	break;// default statements
        }
    }
    if (hasG(command)) {
        switch (command->G)
        {
            case 0: // fast move
            	;
                float xc, yc, thetac;
                xc = robot->odometry->position.x;
                yc = robot->odometry->position.y;
                thetac = robot->odometry->position.theta; // ne pas faire ça en vrai
                if (hasX(command)) xc = command->X;
                if (hasY(command)) yc = command->Y;
                if (hasA(command)){
                    thetac = command->A;
                    fastMoveTheta(robot, xc, yc, thetac);
                }
                else {
                    fastMove(robot, xc, yc);
                }
                robot->controlMode = 0;
                break;
            case 1: // fast move
            	;
                xc = robot->odometry->position.x;
                yc = robot->odometry->position.y;
                thetac = robot->odometry->position.theta; // ne pas faire ça en vrai
                if (hasX(command)) xc = command->X;
                if (hasY(command)) yc = command->Y;
                if (hasA(command)){
                    thetac = command->A;
                    linearMoveTheta(robot, xc, yc, thetac);
                }
                else {
                    linearMove(robot, xc, yc);
                }
                robot->controlMode = 0;
                break;
            case 10:
                if (hasI(command)) robot->linearVelocity = command->I;
                if (hasJ(command)) robot->angularVelocity = command->J;
                robot->controlMode = 1;
                break;
            case 11:
                if (hasI(command)) robot->vitG = command->I;
                if (hasJ(command)) robot->vitD = command->J;
                robot->controlMode = 2;
                break;
            case 92:
            	;
                float x, y, theta;
                x = robot->odometry->position.x;
                y = robot->odometry->position.y;
                theta = robot->odometry->position.theta; // ne pas faire ça en vrai
                if (hasX(command)) x = command->X;
                if (hasY(command)) y = command->Y;
                if (hasA(command)) theta = command->A;
                setPosition(robot, x, y, theta);
                linearMoveTheta(robot, x, y, theta);
                break;

            default:
                break;// default statements
        }
    }
}

/*
 *  deplacement functions setters
 */


void fastMove(diffDriveRobot_Context* robot, float xc, float yc){
	robot->controlMode = 0;
	robot->motionController->consign.x = xc;
    robot->motionController->consign.y = yc;
    robot->motionController->consign.theta = atan2(yc - robot->odometry->position.y, xc - robot->odometry->position.x);
    robot->motionController->movementMode = 0;
}

void fastMoveTheta(diffDriveRobot_Context* robot, float xc, float yc, float thetac){
	robot->controlMode = 0;
	robot->motionController->consign.x = xc;
    robot->motionController->consign.y = yc;
    robot->motionController->consign.theta = thetac;
    robot->motionController->movementMode = 0;
}

void linearMove(diffDriveRobot_Context* robot, float xc, float yc){
	robot->controlMode = 0;
	robot->motionController->consign.x = xc;
    robot->motionController->consign.y = yc;
    robot->motionController->consign.theta = atan2(yc - robot->odometry->position.y, xc - robot->odometry->position.x);
    robot->motionController->movementMode = 1;
}

void linearMoveTheta(diffDriveRobot_Context* robot, float xc, float yc, float thetac){
	robot->controlMode = 0;
    robot->motionController->consign.x = xc;
    robot->motionController->consign.y = yc;
    robot->motionController->consign.theta = thetac;
    robot->motionController->movementMode = 1;
}


void breakeMotors(diffDriveRobot_Context* robot){
    motor_breake(robot->motorD);
    motor_breake(robot->motorG);
}

void unBreakeMotors(diffDriveRobot_Context* robot){
    motor_unbreake(robot->motorD);
    motor_unbreake(robot->motorG);
    // reset des integrateurs des pid
    pid_init(robot->pidD);
    pid_init(robot->pidG);
}


/*
 *  Setters
 */


void setPosition(diffDriveRobot_Context* robot, float x, float y, float theta){
    odometry_init(robot->odometry, x, y, theta);
}


// getter
void printPosition(diffDriveRobot_Context* robot, UART_HandleTypeDef *huart){
    float x,y,theta;
    x = robot->odometry->position.x;
    y = robot->odometry->position.y;
    theta = robot->odometry->position.theta;
    char buffer[190];
    HAL_UART_Transmit(huart, (uint8_t*)buffer, sprintf(buffer,"X %f Y %f A %f\n", x, y, theta), 90000);//s/ @suppress("Float formatting support")
}


void printRspeed(diffDriveRobot_Context* robot, UART_HandleTypeDef *huart){
    float consign, measure, command;
    consign = robot->vitD;
    measure = robot->mesureD;
    command = robot->commandD;
    char buffer[190];
    HAL_UART_Transmit(huart, (uint8_t*)buffer, sprintf(buffer,"consign %f measure Y%f command A%f\n", consign, measure, command), 90000);//s/ @suppress("Float formatting support")
}
void printLspeed(diffDriveRobot_Context* robot, UART_HandleTypeDef *huart){
    float consign, measure, command;
    consign = robot->vitG;
    measure = robot->mesureG;
    command = robot->commandG;
    char buffer[190];
    HAL_UART_Transmit(huart, (uint8_t*)buffer, sprintf(buffer,"consign %f measure Y%f command A%f\n", consign, measure, command), 90000);//s/ @suppress("Float formatting support")
}

void printComTest(UART_HandleTypeDef *huart){
	char buffer[190];
    HAL_UART_Transmit(huart, (uint8_t*)buffer, sprintf(buffer,"Com OK\n"), 90000);
}
void printSelf(UART_HandleTypeDef *huart){
	char buffer[190];
    HAL_UART_Transmit(huart, (uint8_t*)buffer, sprintf(buffer,"CmdBoard\n"), 90000);
}




void setPID_D(diffDriveRobot_Context* robot, float Kp, float Ti, float Td){
    robot->pidD->Kp = Kp;
    robot->pidD->Kp = Ti;
    robot->pidD->Kp = Td;
}

void setPID_G(diffDriveRobot_Context* robot, float Kp, float Ti, float Td){
    robot->pidG->Kp = Kp;
    robot->pidG->Kp = Ti;
    robot->pidG->Kp = Td;
}

void setLOdomWheelDia(diffDriveRobot_Context* robot, float wheelDiameter) {
    robot->odometry->wheelRadiusR = wheelDiameter/2;
}

void setROdomWheelDia(diffDriveRobot_Context* robot, float wheelDiameter) {
    robot->odometry->wheelRadiusR = wheelDiameter/2;
}

void setOdomInnerDist(diffDriveRobot_Context* robot, float innerDist) {
    robot->odometry->distanceBetweenWheels = innerDist;
}

void setMotorInnerDist(diffDriveRobot_Context* robot, float innerDist) {
    robot->distBetweenMotorWheels = innerDist;
    robot->differential->distanceBetweenWheels = innerDist;
}


void setRobotSpeeds(diffDriveRobot_Context* robot, float linearSpeed, float angularSpeed){
	robot->controlMode = 1;
    robot->linearVelocity = linearSpeed;
    robot->angularVelocity = angularSpeed;
}

void setWheelsSpeeds(diffDriveRobot_Context* robot, float rightWheelSpeed, float leftWheelSpeed){
    robot->controlMode = 2;
    robot->vitD = rightWheelSpeed;
    robot->vitG = leftWheelSpeed;
}



//
