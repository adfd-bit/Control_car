/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "stm32f4xx_hal_uart.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "motor.h"
#include "OPS9.h"
#include "math.h"
#include "servo_motor.h"
#include "path_plan.h"
#include "ar_screen.h"
#include "lub_cat.h"
#include <stdbool.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef enum {
    STATE_INIT,
    STATE_NAV,
    STATE_STOP,
} SystemState_t;
//-----------------------------------全局变量----------------------------------*/
uint8_t OPS_redata[OPS9_data_len];
volatile float OPS_angle = 0.0f;
volatile float OPS_X = 0.0f;
volatile float OPS_Y = 0.0f;
uint8_t CAT_redata[CAT_data_len];
volatile int16_t CAT_x = 0;
volatile int16_t CAT_y = 0;
uint8_t ar_data[30];
/*----------------------------------------------调试变量(可删)----------------------------*/
uint8_t receive;
uint8_t pathl[4];
int pt[4];
bool pt_sta = false;
uint8_t hc_os[17];
int goal_x = 0, goal_y = 0, goal_w = 0;
volatile bool re_sta = false;
uint8_t lub_cat_re[4];
volatile bool lubanready = false;
bool luban_finish = false;
uint8_t arm_control_data[5];
bool arm_state = false;
/*-----------------------------------状态变量----------------------------------*/
uint32_t ALL_time = 0;
volatile bool send_data_state = true;
volatile bool opsready = false;
volatile bool catready = false;
volatile bool ar_screen_sta = false; /* true = AR_Screen 接收数据完成标志，主循环用 */
SystemState_t currentState = STATE_INIT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*-----------------------------------电机控制=----------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim1) { // 发送数据至电机
        static int id = 0;
        id++;
        if (id == 5) {
            id = 0;
            motor_go();
            HAL_TIM_Base_Stop_IT(&htim1);
            send_data_state = true;
        } else
            send_motor_speed(id);
    } else if (htim == &htim3) {
        static uint8_t a = 0;
        a++;
        if (a > 14) {
            a = 0;
            HAL_TIM_Base_Stop_IT(htim);
            currentState = STATE_NAV;
        }
    }
}

/*-------------------------------------串口接收事件回调函数-------------------------*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == &huart1) { // 鲁班猫数据处理
        if (CAT_redata[0] == 0xAF && CAT_redata[1] == 0xFA && CAT_redata[2] == 0x0B) {
            if (CAT_redata[9] == 0xCF && CAT_redata[10] == 0xFC) {
                CAT_x = *(int16_t *)&CAT_redata[4];
                CAT_y = *(int16_t *)&CAT_redata[6];
                catready = true;
                HAL_UART_Transmit(&huart5, (uint8_t *)"luban_ready", sizeof("luban_ready") - 1, HAL_MAX_DELAY);
            }
        } else if (CAT_redata[0] == 0xAF && CAT_redata[1] == 0XFA && CAT_redata[2] == 0x0C) {
            if (CAT_redata[10] == 0xCF && CAT_redata[11] == 0xFC) {
                CAT_x = *(int16_t *)&CAT_redata[5];
                CAT_y = *(int16_t *)&CAT_redata[7];
                catready = true;
                HAL_UART_Transmit(&huart5, (uint8_t *)"luban_ready", sizeof("luban_ready") - 1, HAL_MAX_DELAY);
            }
        }
        /* DMA_NORMAL 模式收完一包即停，必须重装接收，否则坐标只收一次 */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, CAT_redata, CAT_data_len);
    } else if (huart == &huart3) { // OPS9数据处理
        if (Size == 28 && OPS_redata[0] == 0x0D && OPS_redata[1] == 0x0A && OPS_redata[26] == 0x0A &&
            OPS_redata[27] == 0x0D) {
            OPS_angle = *(float *)&OPS_redata[2];
            OPS_X = *(float *)&OPS_redata[14];
            OPS_Y = *(float *)&OPS_redata[18];
            float temp = OPS_X;
            OPS_X = OPS_Y;
            OPS_Y = -temp;
            if (OPS_X >= -20000.0f && OPS_X <= 20000.0f && OPS_Y >= -20000.0f && OPS_Y <= 20000.0f &&
                OPS_angle >= -360.0f && OPS_angle <= 360.0f) {
                opsready = true;
            }
        }
        /* DMA_NORMAL 模式收完一包即停，必须重装接收，否则坐标只收一次 */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, OPS_redata, OPS9_data_len);
    } else if (huart == &huart4) { // AR_Screen数据处理
        if (Size > 14) {
            ar_screen_sta = true;
        }
    }
}
/*-------------------------------------串口外设---------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // if (huart == &huart5) { // 全局定位
    //     re_sta = true;
    //     HAL_UART_Receive_IT(&huart5, hc_os, 17);
    // }

    // if(huart == &huart5){//路径规划
    // 	pt_sta = 1;
    // 	pt[0] = pathl[0]-48;
    // 	pt[1] = pathl[1]-48;
    // 	pt[2] = pathl[2]-48;
    // 	pt[3] = pathl[3]-48;
    // 	currentState = STATE_NAV;
    // 	HAL_UART_Receive_IT(&huart5, pathl, sizeof(pathl));//路径规划
    // }

    if (huart == &huart5) { // 机械臂调试
        arm_state = true;
        HAL_UART_Receive_IT(&huart5, arm_control_data, 5);
    }
    // if (huart == &huart5) { // 鲁班猫
    //     lubanready = true;
    //     HAL_UART_Receive_IT(&huart1, lub_cat_re, 4);
    // }
}
/*------------------------------------数据接收错误重启--------------------------------__*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        // 无条件清除所有错误标志
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_ORE);
        // 清空HAL库错误标记
        huart->ErrorCode = HAL_UART_ERROR_NONE;

        // 重启串口接收（此时 HAL 已将 RxState 置为 READY，可安全重装）
        ops9_receive_start();
    }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM4_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_UART4_Init();
    MX_UART5_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();
    /* USER CODE BEGIN 2 */
    //--------------------------------初始化----------------------------------
    servo_init();         // 舵机初始化
    motor_en();           // 电机使能
    ops9_receive_start(); /* 启动 OPS9 接收 DMA */
    //--------------------------------调试----------------------------------
    //	HAL_UART_Receive_IT(&huart5, pathl, sizeof(pathl));//路径规划
    // HAL_UART_Receive_IT(&huart5, hc_os, 17); // 全局定位
    //	HAL_UART_Receive_IT(&huart5, &receive, 1);
    HAL_UART_Receive_IT(&huart5, arm_control_data, 5); // 机械臂调试
    // HAL_UART_Receive_IT(&huart1, lub_cat_re, 4);       // 鲁班猫
    TIM1->ARR = 5000 - 1; // 电机发送数据频率
    uint8_t posit_state = 0;
    //   HAL_TIM_Base_Start_IT(&htim3);//OPS9启动
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        //-----------------调试------------------------
        if (arm_state) {
            arm_state = false;
            if (arm_control_data[0] == 'f') {
                int pulse =
                    (arm_control_data[2] - 48) * 100 + (arm_control_data[3] - 48) * 10 + arm_control_data[4] - 48;
                servo_set_angle(arm_control_data[1] - 48, pulse);
            } else if (arm_control_data[0] == '+') {
                int num = (arm_control_data[2] - 48) * 100 + (arm_control_data[3] - 48) * 10 + arm_control_data[4] - 48;
                if (arm_control_data[1] == '0') {
                    send_motor_place_absolute(0, (uint32_t)num);
                } else {
                    send_motor_place_relative(0, (uint32_t)num);
                }
            } else if (arm_control_data[0] == '-') {
                int num = (arm_control_data[2] - 48) * 100 + (arm_control_data[3] - 48) * 10 + arm_control_data[4] - 48;
                if (arm_control_data[1] == '0') {
                    send_motor_place_absolute(1, (uint32_t)num);
                } else {
                    send_motor_place_relative(1, (uint32_t)num);
                }
            }
        }
        if (lubanready) { // 鲁班猫
            lubanready = false;
            Lub_Cat_receive_start();
            if (lub_cat_re[0] == 'f' && lub_cat_re[3] == 'f') {
                if (lub_cat_re[1] == '1') {
                    Lub_Cat_send_yolo(lub_cat_re[2] - 48);
                    if (cat_centre_calibrate()) {
                        Lub_Cat_send_exit();
                        HAL_UART_Transmit(&huart5, (uint8_t *)"okk", sizeof("okk") - 1, HAL_MAX_DELAY);
                    }
                } else if (lub_cat_re[1] == '2') {
                    Lub_Cat_send_ring();
                    if (cat_centre_calibrate()) {
                        Lub_Cat_send_exit();
                        HAL_UART_Transmit(&huart5, (uint8_t *)"okk", sizeof("okk") - 1, HAL_MAX_DELAY);
                    }
                } else {
                    Lub_Cat_send_material(lub_cat_re[2] - 48);
                    if (cat_centre_calibrate()) {
                        Lub_Cat_send_exit();
                        HAL_UART_Transmit(&huart5, (uint8_t *)"okk", sizeof("okk") - 1, HAL_MAX_DELAY);
                    }
                }
            }
        }
        if (re_sta) { // 全局定位
            re_sta = false;
            goal_x = 0;
            goal_y = 0;
            goal_w = 0;
            // 非停车指令才解析目标点，避免 'p' 被下面的 STATE_NAV 覆盖
            if (hc_os[16] == 'f') {
                const int pow10[4] = {1000, 100, 10, 1};
                for (int i = 0; i < 4; i++) {
                    if (hc_os[4] == '0')
                        goal_x += (hc_os[i] - 48) * pow10[i];
                    else
                        goal_x -= (hc_os[i] - 48) * pow10[i];
                    if (hc_os[10] == '0')
                        goal_y += (hc_os[i + 6] - 48) * pow10[i];
                    else
                        goal_y -= (hc_os[i + 6] - 48) * pow10[i];
                }
                if (hc_os[15] == '0')
                    goal_w = (hc_os[12] - 48) * 100 + (hc_os[13] - 48) * 10 + hc_os[14] - 48;
                else
                    goal_w = -((hc_os[12] - 48) * 100 + (hc_os[13] - 48) * 10 + hc_os[14] - 48);
                if (pid_to_goal(goal_x, goal_y, goal_w)) {
                    HAL_UART_Transmit(&huart5, (uint8_t *)"okk", sizeof("okk") - 1, HAL_MAX_DELAY);
                }
            }
        }

        //-----------------状态机------------------------
        switch (currentState) { // 初始化，
        case STATE_INIT:
            continue;
        case STATE_NAV: {
            if (posit_state == 0 && pid_to_goal(200, 200, 0)) {
                posit_state = 1;
            }
            if (posit_state == 1 && pid_to_path(0, 0, 2, 0)) { // 路径规划
                AR_Screen_Start();
                posit_state = 2;
            }
            if (posit_state == 2 && pid_to_path(2, 0, 4, 2)) {
                posit_state = 3;
            }
            if (posit_state == 3 && pid_to_path(4, 2, 0, 2)) {
                posit_state = 4;
            }
            if (posit_state == 4 && pid_to_path(0, 2, 2, 4)) {
                posit_state = 5;
            }
            if (posit_state == 5 && pid_to_path(2, 4, 4, 2)) {
                posit_state = 6;
            }
            if (posit_state == 6 && pid_to_path(4, 2, 0, 2)) {
                posit_state = 7;
            }
            if (posit_state == 7 && pid_to_path(0, 2, 2, 4)) {
                posit_state = 8;
            }
            if (posit_state == 8 && pid_to_path(2, 4, 0, 0)) {
                posit_state = 0;
                if (pid_to_goal(15, -15, 0))
                    currentState = STATE_STOP;
            } else
                continue;
            break;
        }
        case STATE_STOP: {
            motor_stop();
            // 注意删除
            HAL_UART_Transmit(&huart5, (uint8_t *)"over", sizeof("over") - 1, HAL_MAX_DELAY);
            currentState = STATE_INIT;
            break;
        }
        }
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
