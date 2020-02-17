#include "robot.h"

void robot_init(diffDriveRobot_Context *robot, float x0, float y0, float theta0){
    odometry_init(robot->odometry, x0, y0, theta0);
    motionControl_init(robot->motionController, x0, y0, theta0);
    pid_init(robot->pidD);
    pid_init(robot->pidG);
    motor_init(robot->motorG);
    motor_init(robot->motorD);

    robot->vitD = 0.;
    robot->vitG = 0.; // vitesse consigne du point de contact de la roue et du sol
    robot->commandD = 0.;
    robot->commandG = 0.;
    robot->mesureD = 0.;
    robot->mesureG = 0.; // vitesse mesuree du point de contact de la roue et du sol
    robot->linearVelocity = 0.;
    robot->angularVelocity = 0.;

    robot->mode = 0;
}

void robot_update(diffDriveRobot_Context *robot, float dt){
    // odometry position and displacement update
      odometry_update(robot->odometry);

    // mesures vitesses des roues
      float wheelDistAng = robot->odometry->angularDisplacement * robot->distBetweenMotorWheels/2;
      robot->mesureD = (robot->odometry->linearDisplacement + wheelDistAng)/dt*1000;
      robot->mesureG = (robot->odometry->linearDisplacement - wheelDistAng)/dt*1000;

    // mise a jour de la consigne en vitesse et vitese angulaire.
    if (robot->mode == 0){
        motionControl_update(robot->motionController, robot->odometry->position, &(robot->linearVelocity), &(robot->angularVelocity));
      // Conversion consignes vitesses et vitesses angulaires en vitesses roues gauche et droite et controle qu'on soit dans le carre des vitesses
        differential_update(robot->differential, robot->linearVelocity, robot->angularVelocity, &(robot->vitD), &(robot->vitG));

    }
    else if(robot->mode == 1){
        motionControl_update2(robot->motionController, robot->odometry->position, &(robot->linearVelocity), &(robot->angularVelocity));
      // Conversion consignes vitesses et vitesses angulaires en vitesses roues gauche et droite et controle qu'on soit dans le carre des vitesses
        differential_update(robot->differential, robot->linearVelocity, robot->angularVelocity, &(robot->vitD), &(robot->vitG));

    }
    if (robot->mode == 20){
      // Conversion consignes vitesses et vitesses angulaires en vitesses roues gauche et droite et controle qu'on soit dans le carre des vitesses
        differential_update(robot->differential, robot->linearVelocity, robot->angularVelocity, &(robot->vitD), &(robot->vitG));
    }

    // calcul de la commande moteurs Corrigee par des pid.
      robot->commandD = pid_update(robot->pidD, robot->vitD, robot->mesureD);
      robot->commandG = pid_update(robot->pidG, robot->vitG, robot->mesureG);

    // envoie de la commande aux moteurs
      motor_setSpeed(robot->motorD, (int)robot->commandD);
      motor_setSpeed(robot->motorG, (int)robot->commandG);
}
