#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"

typedef struct {
    TIM_HandleTypeDef   timer; // timer used for PWM generation
    uint32_t            channel; // channel of the timer
    GPIO_TypeDef *      motDir_Port; // direction pin port
    uint16_t            motDir_Pin; // direction pin number
    GPIO_TypeDef *      motBrake_Port; // brake pin port
    uint16_t            motBrake_Pin; // brake pin number
    bool                reverseDir; // reverse direction
    int                 maxPWM; // max motor pwm 
} motor_Context;

void motor_init(motor_Context *motor);
void motor_setSpeed(motor_Context* motor, int speed);
void motor_breake(motor_Context *motor);
void motor_unbreake(motor_Context *motor);


#endif
