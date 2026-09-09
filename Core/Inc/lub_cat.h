/*
 * lub_cat.h
 *
 *  Created on: 2026年9月7日
 *      Author: twyyd
 */

#ifndef INC_LUB_CAT_H_
#define INC_LUB_CAT_H_

#include "usart.h"
#include "motor.h"
#include <stdbool.h>

#define CAT_data_len 14 /* 鲁班猫 接收缓冲区大小 （12字节数据帧 + 2字节余量）*/
#define Pixel_Width_center 320
#define Pixel_Height_center 240
#define ErrTol_cat 5.0

static uint16_t calibrate_time __attribute__((unused)) = 0;
extern volatile float ops_angle;
extern volatile float current_r;
extern volatile bool opsready;
extern volatile bool catready;
extern volatile int16_t CAT_x;
extern volatile int16_t CAT_y;
extern uint8_t CAT_redata[14];

void Lub_Cat_receive_start();
void Lub_Cat_receive_stop();
bool pid_to_cat(float angle_taget);

#endif /* INC_LUB_CAT_H_ */