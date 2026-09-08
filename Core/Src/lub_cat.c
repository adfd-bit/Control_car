/*
 * lub_cat.c
 *
 *  Created on: 2026年09月07日
 *      Author: twyyd
 */
#include "lub_cat.h"

void Lub_Cat_receive_start() { HAL_UARTEx_ReceiveToIdle_DMA(&huart1, CAT_redata, CAT_data_len); }
void Lub_Cat_receive_stop() {
    // 中止接收：HAL 内部会禁 DMAR、abort DMA、关 IDLE/RXNE/PE/ERR 中断、复位状态
    HAL_UART_AbortReceive(&huart1);
}