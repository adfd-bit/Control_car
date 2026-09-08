/*
 * ar_screen.c
 *
 *  Created on: 2026年8月29日
 *      Author: twyyd
 */
#include "ar_screen.h"
#include "usart.h"

/* 在此添加函数实现 */
void AR_Screen_Receive() {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, ar_data, 30);
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
}
void AR_Screen_SendData() { HAL_UART_Transmit(&huart4, ar_data, sizeof(ar_data), HAL_MAX_DELAY); }
void AR_Screen_Start() {
    if (!ar_screen_sta) {
        AR_Screen_Receive();
        while (1) {
            if (ar_screen_sta) {
                AR_Screen_SendData();
                break;
            }
        }
    }
}