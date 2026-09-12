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
/*-------------------------------------鲁班猫通讯协议
功能	数据包
色环检测	AF FA 07 01 01 CF FC
物料-红色	AF FA 08 02 01 03 CF FC
物料-黄色	AF FA 08 02 02 00 CF FC
物料-蓝色	AF FA 08 02 03 01 CF FC
物料-绿色	AF FA 08 02 04 06 CF FC
物料-黑色	AF FA 08 02 05 07 CF FC
物料-浅蓝色	AF FA 08 02 06 04 CF FC
YOLO-one	AF FA 08 03 00 03 CF FC
YOLO-two	AF FA 08 03 01 02 CF FC
YOLO-three	AF FA 08 03 02 01 CF FC
YOLO-所有	AF FA 08 03 FF FC CF FC
退出	AF FA 07 04 04 CF FC

鲁班猫四 → 上位机
功能	完整数据包 (hex)
色环结果	AF FA 0B 01 xL xH yL yH CS CF FC
物料结果	AF FA 0B 02 xL xH yL yH CS CF FC
YOLO结果	AF FA 0C 04 cls xL xH yL yH CS CF FC
xL/xH = x 坐标 int16 小端，CS = 数据区异或校验和
AF FA 0C 03 cls xL xH yL yH CS CF FC
            ↑  ↑   ↑  └─────────┘  └─┘
              │  │   │     x,y 坐标   校验和
           │  │   └────────────── 类别 (0/1/2)
           │  └────────────────── 命令 0x03
           └───────────────────── 总长度 = 6 + 6 = 12 = 0x0C
校验和 = 0x03 ^ cls ^ xL ^ xH ^ yL ^ yH
--------------------------------------------------*/
/*-------------------------------------发送指令给鲁班猫------------------------------
发送帧格式：AF FA | LEN | CMD | DATA... | CS | CF FC
  LEN ：整帧字节数 = 7 + 数据字节数（无数据 = 0x07，1 字节数据 = 0x08）
  CS  ：命令字 CMD 与所有 DATA 字节的异或（不含 AF FA、LEN、CF FC）

例：物料-红色 = AF FA 08 02 01 03 CF FC
    LEN = 0x08（7 + 1 字节数据），CMD = 0x02，DATA = 0x01，CS = 0x02 ^ 0x01 = 0x03

huart1 只配了接收 DMA（DMA2_Stream2），没有发送 DMA，所以发送用阻塞方式；
一帧 8 字节 @115200 约 0.7ms，超时给 100ms 足够，避免异常时死等。
注意：裸奔式发送，函数内不做重发/排队，调用节奏由业务层控制（别在中断里调用）。
----------------------------------------------------------------------------------*/
void Lub_Cat_send(uint8_t cmd, const uint8_t *data, uint8_t data_len) {
    uint8_t frame[CAT_FRAME_FIX_LEN + CAT_TX_MAX_DATA]; /* 最长 7 + 4 = 11 字节 */
    uint8_t cs = cmd;                                   /* 校验和初值就是命令字本身 */
    uint8_t i;

    if (data_len > CAT_TX_MAX_DATA) /* 数据区超长直接丢弃，避免发出去一帧错的 */
        return;

    frame[0] = 0xAF; /* 帧头 */
    frame[1] = 0xFA;
    frame[2] = CAT_FRAME_FIX_LEN + data_len; /* LEN = 整帧长度 */
    frame[3] = cmd;                          /* 命令字 */
    for (i = 0; i < data_len; i++) {
        frame[4 + i] = data[i]; /* 数据区 */
        cs ^= data[i];          /* 边填充边累加异或校验 */
    }
    frame[4 + data_len] = cs;   /* 校验和 */
    frame[5 + data_len] = 0xCF; /* 帧尾 */
    frame[6 + data_len] = 0xFC;

    HAL_UART_Transmit(&huart1, frame, CAT_FRAME_FIX_LEN + data_len, 100);
}

/* 色环检测：AF FA 07 01 01 CF FC */
void Lub_Cat_send_ring(void) { Lub_Cat_send(CAT_CMD_RING, 0, 0); }

/* 物料识别：color 传 CAT_Color_t，如 CAT_COLOR_RED -> AF FA 08 02 01 03 CF FC */
void Lub_Cat_send_material(CAT_Color_t color) {
    uint8_t d = (uint8_t)color; /* 临时变量取地址，保证数据区是 1 字节 */
    Lub_Cat_send(CAT_CMD_MATERIAL, &d, 1);
}

/* YOLO 识别：id 传 CAT_Yolo_t，如 CAT_YOLO_ALL -> AF FA 08 03 FF FC CF FC */
void Lub_Cat_send_yolo(CAT_Yolo_t id) {
    uint8_t d = (uint8_t)id;
    Lub_Cat_send(CAT_CMD_YOLO, &d, 1);
}

/* 退出：AF FA 07 04 04 CF FC */
void Lub_Cat_send_exit(void) { Lub_Cat_send(CAT_CMD_EXIT, 0, 0); }
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
    angle_error = angle_taget - OPS_angle;

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
        if (HAL_GetTick() - ALL_time > 100) {
            int len = sprintf(c, "%d,%d,%f\n", CAT_x, CAT_y, OPS_angle);
            HAL_UART_Transmit(&huart5, (uint8_t *)c, len, HAL_MAX_DELAY);
            ALL_time = HAL_GetTick();
        }

        HAL_UART_Transmit(&huart5, (uint8_t *)c, sizeof(c), HAL_MAX_DELAY);

        if (pid_to_cat(current_r)) {
            motor_stop();
            return true;
        }
    }
}