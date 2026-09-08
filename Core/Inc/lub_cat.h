/*
 * lub_cat.h
 *
 *  Created on: 2026年9月7日
 *      Author: twyyd
 */

#ifndef INC_LUB_CAT_H_
#define INC_LUB_CAT_H_

#include "usart.h"

#define CAT_data_len 14 /* 鲁班猫 接收缓冲区大小 （12字节数据帧 + 2字节余量）*/
#define Pixel_Width_center 320
#define Pixel_Height_center 240

extern volatile int16_t CAT_x;
extern volatile int16_t CAT_y;

extern uint8_t CAT_redata[14];

void Lub_Cat_receive_start();
void Lub_Cat_receive_stop();

#endif /* INC_LUB_CAT_H_ */