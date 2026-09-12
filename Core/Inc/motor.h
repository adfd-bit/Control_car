/*
 * motor.h
 *
 *  Created on: 2026年6月21日
 *      Author: twyyd
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include "tim.h"
#include "stdbool.h"
#include "usart.h"
#include "math.h"
#include "path_plan.h"

#define PI 3.14159265358979323846
#define ErrTol 10.0
#define ErrTol_r 3.0
#define kI_max 20000
#define KI_min -20000
#define mov_max 320
#define mov_min -320
#define rad_max 900
#define rad_min -900
#define va_max 160
extern uint32_t ALL_time;

extern volatile float OPS_angle;
extern volatile float OPS_X;
extern volatile float OPS_Y;
extern float angle_OLD;
extern float X_OLD;
extern float Y_OLD;
extern volatile bool send_data_state;
extern int path_length;
extern PathPoint path[25];
extern Node nodes[MAP_SIZE][MAP_SIZE];
static uint16_t arrive_time __attribute__((unused)) = 0;
extern volatile bool opsready;
void send_motor_place_absolute(uint8_t dir, uint32_t pulse);
void send_motor_place_relative(uint8_t dir, uint32_t pulse);
void send_motor_speed(uint8_t id);
void motor_go();
void motor_enable(uint8_t id);
void motor_en();
void motor_p();
void motor_stop();
void Speed_Conversion(double *vx, double *vy);
void control_v(double vx, double vy, double wv);
bool pid_to_x_y(float X_target, float Y_target);
bool pid_to_w(float angle_taget);
bool pid_to_v(float X_target, float Y_target, float angle_taget);
bool pid_to_goal(float X_target, float Y_target, float angle_taget);
bool pid_to_goal_relative(float X_target, float Y_target, float angle_taget);
bool pid_to_path(int sta_x, int sta_y, int goal_x, int goal_y);

void send_motor_place(uint8_t dir, uint32_t pulse);
#endif /* INC_MOTOR_H_ */
