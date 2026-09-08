/*
 * servo_motor.c
 *
 *  Created on: 2026年8月9日
 *      Author: twyyd
 */

#include "servo_motor.h"

void servo_init() {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}
void servo_stop() {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 500);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 500);
    HAL_Delay(1);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}
static uint32_t angle_to_pulse(uint16_t angle, uint16_t min, uint16_t max) {
    angle = (angle > max) ? max : (angle < min) ? min : angle;
    uint32_t pulse;
    pulse = (float)(angle - min) / (float)(max - min) * (servo_max - servo_min) + servo_min;
    return pulse;
}
void servo_set_angle(Servo_ID id, uint16_t angle) {
    switch (id) {
    case SERVO_SG90:
        TIM4->CCR1 = angle_to_pulse(angle, sg90_min, sg90_max);
        break;
    case SERVO_XH270:
        TIM4->CCR2 = angle_to_pulse(angle, XH270_min, XH270_max);
        break;
    case SERVO_XH360:
        TIM4->CCR3 = angle_to_pulse(angle, XH360_min, XH360_max);
        break;
    }
}
