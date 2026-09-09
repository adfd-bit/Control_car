/*
 * ar_screen.h
 *
 *  Created on: 2026年8月29日
 *      Author: twyyd
 */

#ifndef INC_AR_SCREEN_H_
#define INC_AR_SCREEN_H_

#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "usart.h"

extern uint8_t ar_data[30];         // 声明全局变量 ar_data
extern volatile bool ar_screen_sta; // 声明全局变量 ar_screen_sta
/* 在此添加函数声明 */
void AR_Screen_Receive();
void AR_Screen_SendData();
void AR_Screen_Start();
void AR_screen_stop();
#endif /* INC_AR_SCREEN_H_ */
