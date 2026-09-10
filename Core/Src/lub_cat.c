/*
 * lub_cat.c
 *
 *  Created on: 2026年09月07日
 *      Author: twyyd
 */
#include "lub_cat.h"
#include <math.h>

int16_t error_x = 0;
int16_t error_y = 0;
int16_t error_x_last = 0;
int16_t error_y_last = 0;
int16_t error_x_sum = 0;
int16_t error_y_sum = 0;
float angle_error = 0;
float angle_error_last = 0;
float angle_error_sum = 0;

void Lub_Cat_receive_start() { HAL_UARTEx_ReceiveToIdle_DMA(&huart1, CAT_redata, CAT_data_len); }
void Lub_Cat_receive_stop() {
    // 中止接收：HAL 内部会禁 DMAR、abort DMA、关 IDLE/RXNE/PE/ERR 中断、复位状态
    HAL_UART_AbortReceive(&huart1);
}
bool pid_to_cat(float angle_taget) {
    if (!catready || !opsready)
        return false;
    catready = false;
    opsready = false;

    float Kp = 0.1f; // x轴比例增益
    float Kd = 0.0f; // x轴微分增益
    float ki = 0.0f; // x轴积分增益
    double vx = 0;
    double vy = 0;
    double wv = 0;

    error_x = Pixel_Width_center - CAT_x;
    error_y = Pixel_Height_center - CAT_y;
    angle_error = angle_taget - ops_angle;

    vy = (double)(Kp * error_x + Kd * (error_x - error_x_last) + ki * error_x_sum);
    vx = (double)(Kp * error_y + Kd * (error_y - error_y_last) + ki * error_y_sum);
    wv = (double)(2 * angle_error + 1 * (angle_error - angle_error_last) + 0.006 * angle_error_sum);

    vx = vx > mov_max ? mov_max : vx > mov_min ? vx : mov_min;
    vy = vy > mov_max ? mov_max : vy > mov_min ? vy : mov_min;
    wv = wv > rad_max ? rad_max : wv > rad_min ? wv : rad_min;

    Speed_Conversion(&vx, &vy);
    control_v(vx, vy, wv);

    error_x_sum += error_x;
    error_y_sum += error_y;
    angle_error_sum += angle_error;
    error_x_last = error_x;
    error_y_last = error_y;
    angle_error_last = angle_error;

    if (send_data_state) {
        send_data_state = false;
        TIM1->CNT = 0;
        HAL_TIM_Base_Start_IT(&htim1);
    }

    error_x_sum = error_x_sum > kI_max ? kI_max : error_x_sum > KI_min ? error_x_sum : KI_min;
    error_y_sum = error_y_sum > kI_max ? kI_max : error_y_sum > KI_min ? error_y_sum : KI_min;
    angle_error_sum = angle_error_sum > kI_max ? kI_max : angle_error_sum > KI_min ? angle_error_sum : KI_min;

    if (error_x > -ErrTol_cat && error_x < ErrTol_cat && error_y > -ErrTol_cat && error_y < ErrTol_cat &&
        angle_error > -ErrTol_r && angle_error < ErrTol_r) {
        calibrate_time++;
        if (calibrate_time > 20) {
            error_x_last = 0;
            error_y_last = 0;
            angle_error_last = 0;
            error_x_sum = 0;
            error_y_sum = 0;
            angle_error_sum = 0;
            calibrate_time = 0;
            current_r = angle_taget;
            return true;
        } else
            return false;
    } else {
        calibrate_time = 0;
        return false;
    }
}
bool cat_centre_calibrate() {
    char c[40];
    while (1) {
        // 注意删除
        sprintf(c, "CAT_x: %d, CAT_y: %d, OPS_angle: %f\n", CAT_x, CAT_y, OPS_angle);
        hal_uart_transmit(&huart5, (uint8_t *)c, strlen(c), HAL_MAX_DELAY);

        if (pid_to_cat(current_r)) {
            motor_stop();
            return true;
        }
    }
}