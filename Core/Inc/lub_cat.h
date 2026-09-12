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
#include "stdio.h"

#define CAT_data_len 14 /* 鲁班猫 接收缓冲区大小 （12字节数据帧 + 2字节余量）*/
#define Pixel_Width_center 320
#define Pixel_Height_center 240
#define ErrTol_cat 5.0

/*-------------------------------发送指令给鲁班猫（协议定义）------------------------*/
#define CAT_FRAME_FIX_LEN 7 /* 帧固定开销：AF FA + LEN + CMD + CS + CF FC */
#define CAT_TX_MAX_DATA 4   /* 数据区最大字节数 */

#define CAT_CMD_RING 0x01     /* 色环检测 */
#define CAT_CMD_MATERIAL 0x02 /* 物料识别 */
#define CAT_CMD_YOLO 0x03     /* YOLO 识别 */
#define CAT_CMD_EXIT 0x04     /* 退出 */

/* 物料颜色编号（对应协议中的 物料-xxx） */
typedef enum {
    CAT_COLOR_RED = 0x01,        /* 物料-红色   AF FA 08 02 01 03 CF FC */
    CAT_COLOR_YELLOW = 0x02,     /* 物料-黄色   AF FA 08 02 02 00 CF FC */
    CAT_COLOR_BLUE = 0x03,       /* 物料-蓝色   AF FA 08 02 03 01 CF FC */
    CAT_COLOR_GREEN = 0x04,      /* 物料-绿色   AF FA 08 02 04 06 CF FC */
    CAT_COLOR_BLACK = 0x05,      /* 物料-黑色   AF FA 08 02 05 07 CF FC */
    CAT_COLOR_LIGHT_BLUE = 0x06, /* 物料-浅蓝色 AF FA 08 02 06 04 CF FC */
} CAT_Color_t;

/* YOLO 目标编号 */
typedef enum {
    CAT_YOLO_ONE = 0x00,   /* YOLO-one   AF FA 08 03 00 03 CF FC */
    CAT_YOLO_TWO = 0x01,   /* YOLO-two   AF FA 08 03 01 02 CF FC */
    CAT_YOLO_THREE = 0x02, /* YOLO-three AF FA 08 03 02 01 CF FC */
    CAT_YOLO_ALL = 0xFF,   /* YOLO-所有  AF FA 08 03 FF FC CF FC */
} CAT_Yolo_t;

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
void Lub_Cat_send(uint8_t cmd, const uint8_t *data, uint8_t data_len);
void Lub_Cat_send_ring(void);
void Lub_Cat_send_material(CAT_Color_t color);
void Lub_Cat_send_yolo(CAT_Yolo_t id);
void Lub_Cat_send_exit(void);
bool pid_to_cat(float angle_taget);

#endif /* INC_LUB_CAT_H_ */