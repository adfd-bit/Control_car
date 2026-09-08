/*
 * servo_motor.h
 *
 *  Created on: 2026年8月9日
 *      Author: twyyd
 */

#ifndef INC_SERVO_MOTOR_H_
#define INC_SERVO_MOTOR_H_

#include"tim.h"

#define servo_max 2500
#define servo_min 500
#define sg90_max 180
#define sg90_min 0
#define XH270_max 270
#define XH270_min 0
#define XH360_max 360
#define XH360_min 0

typedef enum {
    SERVO_SG90  = 1,
    SERVO_XH270 = 2,
    SERVO_XH360 = 3
} Servo_ID;

void servo_init();
void servo_stop();
void servo_set_angle(Servo_ID id, uint16_t angle);

#endif /* INC_SERVO_MOTOR_H_ */
