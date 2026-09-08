/*
 * OPS9.c
 *
 *  Created on: 2026年6月21日
 *      Author: twyyd
 */

#include "OPS9.h"
void ops9_receive_start() { HAL_UARTEx_ReceiveToIdle_DMA(&huart3, OPS_redata, OPS9_data_len); }
void ops9_receive_stop() {
    // 中止接收：HAL 内部会禁 DMAR、abort DMA、关 IDLE/RXNE/PE/ERR 中断、复位状态
    HAL_UART_AbortReceive(&huart3);
}
void set_cur_pos(float angle, float x, float y) {
    uint8_t setdata[16];
    setdata[0] = 'A';
    setdata[1] = 'C';
    setdata[2] = 'T';
    setdata[3] = 'A';
    memcpy(&setdata[4], &angle, sizeof(angle));
    memcpy(&setdata[8], &x, sizeof(x));
    memcpy(&setdata[12], &y, sizeof(y));
    HAL_UART_Transmit(&huart3, setdata, sizeof(setdata), HAL_MAX_DELAY);
}
