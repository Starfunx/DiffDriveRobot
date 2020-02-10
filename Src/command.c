#include "command.h"


void peekCommand(gcodeCommand_context* command, diffDriveRobot_Context* robot){
    if (hasM(command)) {
        switch (command->M)
        {
            case 18:
            	breakeMotors(robot);
              	break;
            case 19:
            	unbreakeMotors(robot);
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
            default:
                break;// default statements
        }
    }
}


void fastMove(diffDriveRobot_Context* robot, float xc, float yc, float thetac){
    robot->motionController->consign.x = xc;
    robot->motionController->consign.y = yc;
    robot->motionController->consign.theta = thetac;
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
