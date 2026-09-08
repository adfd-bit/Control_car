/*
 * OPS9.h
 *
 *  Created on: 2026年6月21日
 *      Author: twyyd
 */

#ifndef INC_OPS9_H_
#define INC_OPS9_H_

#include "usart.h"
#include "string.h"
#include "stdbool.h"

#define OPS9_data_len 30 /* OPS9 接收缓冲区大小（28字节数据帧 + 2字节余量） */
extern uint8_t OPS_redata[OPS9_data_len];

extern volatile float OPS_angle;
extern volatile float OPS_X;
extern volatile float OPS_Y;
extern uint32_t ALL_time;

void ops9_receive_start();
void ops9_receive_stop();
void set_cur_pos(float angle, float x, float y);

#endif /* INC_OPS9_H_ */
