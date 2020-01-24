#include "motor.h"

void motor_init(motor_Context *motor){
    setPWM(motor->timer, motor->channel, 255, 0); // reset PWM
    HAL_GPIO_WritePin(motor->motDir_Port, motor->motDir_Pin, GPIO_PIN_RESET); //
    HAL_GPIO_WritePin(motor->motBrake_Port, motor->motBrake_Pin, GPIO_PIN_RESET);
}

void motor_setSpeed(motor_Context *motor, int speed){
    if (motor->reverseDir){
        speed = -speed;
    }
    if (speed < 0){
        speed = -speed;
        HAL_GPIO_WritePin(motor->motDir_Port, motor->motDir_Pin, GPIO_PIN_SET);
    }
    else {
        HAL_GPIO_WritePin(motor->motDir_Port, motor->motDir_Pin, GPIO_PIN_RESET);
    }

    if (speed > motor->maxPWM){
        speed = motor->maxPWM;
    }

    setPWM(motor->timer, motor->channel, 255, speed); // set PWM
}

void motor_breake(motor_Context *motor){
    HAL_GPIO_WritePin(motor->motBrake_Port, motor->motBrake_Pin, GPIO_PIN_SET);
}

void motor_unbreake(motor_Context *motor){
    HAL_GPIO_WritePin(motor->motBrake_Port, motor->motBrake_Pin, GPIO_PIN_RESET);
}

void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t pulse) {
	  HAL_TIM_PWM_Stop(&timer, channel);    // stop generation of pwm
	  TIM_OC_InitTypeDef sConfigOC;   timer.Init.Period = period;           // set the period duration
	  HAL_TIM_PWM_Init(&timer);  // reinititialise with new period value
	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse = pulse;              // set the pulse duration
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
	  HAL_TIM_PWM_Start(&timer, channel);   // start pwm generation}
}
