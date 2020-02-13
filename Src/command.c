#include "command.h"


void peekCommand(gcodeCommand_context* command, diffDriveRobot_Context* robot, UART_HandleTypeDef *huart){
    if (hasM(command)) {
        float Kp,Ti,Td;
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
                if (hasA(command)) thetac = command->A;
                fastMove(robot, xc, yc, thetac);
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
                break;

            default:
                break;// default statements
        }
    }
}


void fastMove(diffDriveRobot_Context* robot, float xc, float yc){
    robot->motionController->consign.x = xc;
    robot->motionController->consign.y = yc;
    robot->motionController->consign.theta = thetac;
}

void fastMoveTheta(diffDriveRobot_Context* robot, float xc, float yc, float thetac){

}

void LinearMove(diffDriveRobot_Context* robot, float xc, float yc){

}

void LinearMoveTheta(diffDriveRobot_Context* robot, float xc, float yc, float thetac){

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

void setPosition(diffDriveRobot_Context* robot, float x, float y, float theta){
    odometry_init(robot->odometry, x, y, theta);
}

void printPosition(diffDriveRobot_Context* robot, UART_HandleTypeDef *huart){
    float x,y,theta;
    x = robot->odometry->position.x;
    y = robot->odometry->position.y;
    theta = robot->odometry->position.theta;
    char buffer[190];
    HAL_UART_Transmit(huart, (uint8_t*)buffer, sprintf(buffer,"X%f Y%f A%f\n", x, y, theta), 90000);//s/ @suppress("Float formatting support")
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
