/*
 * motor.c
 *
 *  Created on: 2026年6月21日
 *      Author: twyyd
 */
#include "motor.h"
float current_x = 0;
float current_y = 0;
float current_r = 0;
float angle_OLD = 0;
float X_OLD = 0;
float Y_OLD = 0;
float ang_sum = 0;
float xki_sum = 0;
float yki_sum = 0;
float vxa = 0;
float vya = 0;
int motor[4];
void motor_enable(uint8_t id) {
    uint8_t data[6] = {id, 0xF3, 0xAB, 0x01, 0x00, 0x6B};
    HAL_UART_Transmit(&huart2, data, sizeof(data), 100);
    HAL_Delay(10);
}
void motor_en() {
    motor_enable(1);
    motor_enable(2);
    motor_enable(3);
    motor_enable(4);
}
void send_motor_place_absolute(uint8_t dir, uint32_t pulse) {
    pulse = pulse * 3200 / 36;
    uint8_t data[13];
    data[0] = 5;
    data[1] = 0xFD;
    data[2] = dir;
    data[3] = 0x00;
    data[4] = 0x3C;
    data[5] = 0x00;
    data[6] = pulse >> 24;
    data[7] = pulse >> 16;
    data[8] = pulse >> 8;
    data[9] = pulse;
    data[10] = 0x01;
    data[11] = 0x00;
    data[12] = 0x6B;
    HAL_UART_Transmit(&huart2, data, sizeof(data), HAL_MAX_DELAY);
}
void send_motor_place_relative(uint8_t dir, uint32_t pulse) {
    pulse = pulse * 3200 / 36;
    uint8_t data[13];
    data[0] = 5;
    data[1] = 0xFD;
    data[2] = dir;
    data[3] = 0x00;
    data[4] = 0x3C;
    data[5] = 0x00;
    data[6] = pulse >> 24;
    data[7] = pulse >> 16;
    data[8] = pulse >> 8;
    data[9] = pulse;
    data[10] = 0x00;
    data[11] = 0x00;
    data[12] = 0x6B;
    HAL_UART_Transmit(&huart2, data, sizeof(data), HAL_MAX_DELAY);
}
void send_motor_speed(uint8_t id) {
    uint8_t data[8];
    data[0] = id;
    data[1] = 0xF6;
    data[2] = (motor[id - 1] > 0) ? 0x01 : 0x00;
    data[3] = (motor[id - 1] > 0) ? (uint8_t)(motor[id - 1] >> 8) : (uint8_t)(-motor[id - 1] >> 8);
    data[4] = (motor[id - 1] > 0) ? (uint8_t)motor[id - 1] : (uint8_t)(-motor[id - 1]);
    data[5] = 0x00;
    data[6] = 0x01;
    data[7] = 0x6B;
    HAL_UART_Transmit(&huart2, data, 8, HAL_MAX_DELAY);
}
void motor_go() {
    uint8_t c[4] = {0x00, 0xFF, 0x66, 0x6B};
    HAL_UART_Transmit(&huart2, c, 4, HAL_MAX_DELAY);
}
void motor_stop() {
    HAL_Delay(30);
    uint8_t data[5] = {0x00, 0xFE, 0x98, 0x00, 0x6B};
    HAL_UART_Transmit(&huart2, data, sizeof(data), HAL_MAX_DELAY);
}
void Speed_Conversion(double *vx, double *vy) {
    double ops_rad = (double)OPS_angle * PI / 180.0;
    double temp_vx = *vx;
    double temp_vy = *vy;
    *vx = sin(ops_rad) * temp_vy + cos(ops_rad) * temp_vx;
    *vy = cos(ops_rad) * temp_vy - sin(ops_rad) * temp_vx;
}
void control_v(double vx, double vy, double wv) { // 转换成对应的电机的速度 X型
    motor[0] = (int)(vy + vx - 0.6f * wv);
    motor[1] = (int)(vy - vx + 0.6f * wv);
    motor[2] = (int)(vy - vx - 0.6f * wv);
    motor[3] = (int)(vy + vx + 0.6f * wv);
}
// void control_v(double vx,double vy,double wv){ // 转换成对应的电机的速度 O形
//	motor[0] = (int)(vy - vx - 0.6f * wv);
//	motor[1] = (int)(vy + vx + 0.6f * wv);
//	motor[2] = (int)(vy + vx - 0.6f * wv);
//	motor[3] = (int)(vy - vx + 0.6f * wv);
// }

bool pid_to_v(float X_target, float Y_target, float angle_taget) { // 位移到对应坐标
    float kp = 0.6;                                                // pid参数可调
    float kd = 0.48;
    float ki = 0.0003;
    double vx;
    double vy;
    double wv;
    if (!opsready)
        return false;
    opsready = false;
    float ops_cache = OPS_angle;
    if (angle_taget == 180 && ops_cache < 0) {
        ops_cache = ops_cache + 360;
    }
    if (angle_taget == -180 && ops_cache > 0) {
        ops_cache = ops_cache - 360;
    }
    if (X_OLD == 0.0) {
        vx = (double)(kp * (X_target - OPS_X) + ki * (xki_sum));
        vy = (double)(kp * (Y_target - OPS_Y) + ki * (yki_sum));
        wv = (double)((kp + 0.05) * (angle_taget - ops_cache) + (ki + 0.01) * (ang_sum));
    } else {
        vx = (double)(kp * (X_target - OPS_X) + kd * (X_OLD - OPS_X) + ki * (xki_sum));
        vy = (double)(kp * (Y_target - OPS_Y) + kd * (Y_OLD - OPS_Y) + ki * (yki_sum));
        if ((int)angle_taget == (int)current_r)
            wv = (double)((kp + 9.0) * (angle_taget - ops_cache) + (kd + 0.3) * (angle_OLD - ops_cache) +
                          (ki + 0.0064) * (ang_sum)); // 边走边转
        else
            wv = (double)((kp + 2.0) * (angle_taget - ops_cache) + (kd + 0.3) * (angle_OLD - ops_cache) +
                          (ki + 0.0064) * (ang_sum)); // 纯转角度
    }

    if (abs((int)(vx - vxa)) > va_max && abs(vx) > abs(vxa)) {
        if (vx > 0)
            vx = vxa + va_max;
        else
            vx = vxa - va_max;
    }
    if (abs((int)(vy - vya)) > va_max && abs(vy) > abs(vya)) {
        if (vy > 0)
            vy = vya + va_max;
        else
            vy = vya - va_max;
    }

    vxa = vx;
    vya = vy;

    vx = vx > mov_max ? mov_max : vx > mov_min ? vx : mov_min;
    vy = vy > mov_max ? mov_max : vy > mov_min ? vy : mov_min;
    wv = wv > rad_max ? rad_max : wv > rad_min ? wv : rad_min;

    Speed_Conversion(&vx, &vy);
    control_v(vx, vy, wv);

    xki_sum += (X_target - OPS_X);
    yki_sum += (Y_target - OPS_Y);
    ang_sum += (angle_taget - ops_cache);
    angle_OLD = ops_cache;
    X_OLD = OPS_X;
    Y_OLD = OPS_Y;
    xki_sum = xki_sum > kI_max ? kI_max : xki_sum > KI_min ? xki_sum : KI_min;
    yki_sum = yki_sum > kI_max ? kI_max : yki_sum > KI_min ? yki_sum : KI_min;
    ang_sum = ang_sum > kI_max ? kI_max : ang_sum > KI_min ? ang_sum : KI_min;
    if (send_data_state) {
        send_data_state = false;
        TIM1->CNT = 0;
        HAL_TIM_Base_Start_IT(&htim1);
    }
    if ((X_target - OPS_X) > -ErrTol && (X_target - OPS_X) < ErrTol && (Y_target - OPS_Y) > -ErrTol &&
        (Y_target - OPS_Y) < ErrTol && (angle_taget - OPS_angle) > -ErrTol_r && (angle_taget - OPS_angle) < ErrTol_r) {
        arrive_time++;
        if (arrive_time > 6) {
            X_OLD = 0;
            Y_OLD = 0;
            angle_OLD = 0;
            xki_sum = 0;
            yki_sum = 0;
            ang_sum = 0;
            arrive_time = 0;
            vxa = 0;
            vya = 0;
            current_x = X_target;
            current_y = Y_target;
            current_r = angle_taget;
            return true;
        } else
            return false;
    } else {
        arrive_time = 0;
        return false;
    }
}
bool pid_to_goal(float X_target, float Y_target, float angle_taget) { // 绝对坐标
    char c[40];
    while (1) {
        //		注意删除
        if (HAL_GetTick() - ALL_time > 100) {
            int len = sprintf(c, "%f,%f,%f\n", OPS_X, OPS_Y, OPS_angle);
            HAL_UART_Transmit(&huart6, (uint8_t *)c, len, HAL_MAX_DELAY);
            ALL_time = HAL_GetTick();
        }

        if (pid_to_v(X_target, Y_target, angle_taget)) {
            motor_stop();
            return true;
        }
    }
}
bool pid_to_goal_relative(float X_target, float Y_target, float angle_taget) { // 相对坐标
    float X = current_x + X_target;
    float Y = current_y + Y_target;
    float angle = current_r + angle_taget;
    if (angle > 180)
        angle -= 360;
    else if (angle < -180)
        angle += 360;
    return pid_to_goal(X, Y, angle);
}
bool pid_to_path(int sta_x, int sta_y, int goal_x, int goal_y) { // Astar路径规划
    static float old = 0;
    float angle = 0;
    bool state;
    char c[40];
    if (!Find_path(sta_x, sta_y, goal_x, goal_y)) {
        return false;
    }
    for (uint8_t j = 2; j < path_length; j += 2) {
        angle = -90 * (path[j].x - path[j - 2].x) / 2; // 转动到Y轴
        if (abs((int)(angle - old)) == 180)
            angle = old;
        pid_to_goal(current_x, current_y, angle);
        old = angle;
        if (j + 2 < path_length &&
            (abs(path[j + 2].y - path[j - 2].y) == 4 || abs(path[j + 2].x - path[j - 2].x) == 4)) {
            j += 2;
        }
        state = false;
        while (!state) {
            // 注意删除
            if (HAL_GetTick() - ALL_time > 100) {
                int len = sprintf(c, "%f,%f,%f\n", OPS_X, OPS_Y, OPS_angle);
                HAL_UART_Transmit(&huart6, (uint8_t *)c, len, HAL_MAX_DELAY);
                ALL_time = HAL_GetTick();
            }

            if (pid_to_goal(nodes[path[j].x][path[j].y].real_x, nodes[path[j].x][path[j].y].real_y, angle)) {
                state = true;
            }
        }
    }
    return true;
}
/*====================================================================
 * ↓↓↓ 非阻塞版定位判断（状态机实现）—— 确认后替换上面的阻塞版 ↓↓↓
 * ------------------------------------------------------------------
 * 设计思路：
 *  1. 阻塞版的本质是 while(1){ 不断调用 pid_to_v() 直到返回 true }。
 *     pid_to_v() 内部靠 opsready（每收到一帧 OPS9 有效数据置1）限速，
 *     所以非阻塞版每次调用只执行"一步"pid_to_v，效果与阻塞版等价，
 *     只是把"等"的时间还给主循环（串口指令、OPS9 超时检测不再被卡住）。
 *  2. pid_to_path_nb() 用静态状态机记住当前走到第几个节点：
 *     NB_IDLE(规划) → NB_TURN(原地转向) → NB_MOVE(走向节点) → NB_DONE
 *     主循环每圈调用一次，返回 true 表示整段路径走完。
 *  3. 任务参数变化（或上次已完成）时自动重新规划，重复走同一路线也 OK。
 * ------------------------------------------------------------------
 * 启用步骤：
 *  1) 取消本块注释，删除/注释掉上面的 pid_to_goal 和 pid_to_path
 *  2) motor.h 中追加声明：
 *     bool pid_to_goal_nb(float X_target,float Y_target,float angle_taget);
 *     bool pid_to_path_nb(int sta_x,int sta_y,int goal_x,int goal_y);
 *  3) main.c 的 STATE_NAV 必须改成 switch(posit_state) 写法（见本块末尾示例）：
 *     原来的 if 链在非阻塞下会每圈重复进入前面几步，不能直接用！
 *====================================================================*/
/*
typedef enum {
        NB_IDLE = 0,   // 待开始：调用 Find_path 规划
        NB_TURN,       // 原地转向到下一段方向
        NB_MOVE,       // 沿当前方向走向下一个节点
        NB_DONE        // 整段路径走完
} nb_state_t;

static nb_state_t nb_state = NB_IDLE;
static uint8_t nb_j = 0;        // 当前目标节点在 path 中的下标
static float nb_old = 0;        // 上一段的角度（用于 180° 修正）
static float nb_angle = 0;      // 当前段的角度
static int nb_sta_x, nb_sta_y, nb_goal_x, nb_goal_y; // 记录本次任务参数

static void nb_debug_send(void){// 原阻塞循环里的 100ms 调试输出（注意删除）
        char c[40];
        if(HAL_GetTick() - ALL_time > 100){
                int len = sprintf(c,"%f,%f,%f\n",OPS_X,OPS_Y,OPS_angle);
                HAL_UART_Transmit(&huart6, (uint8_t*)c, len, HAL_MAX_DELAY);
                ALL_time = HAL_GetTick();
        }
}

bool pid_to_goal_nb(float X_target,float Y_target,float angle_taget){
        // 阻塞版就是 while(1){ pid_to_v() }，非阻塞版每次调用只推进一步
        nb_debug_send();
        return pid_to_v(X_target, Y_target, angle_taget);
}

bool pid_to_path_nb(int sta_x,int sta_y,int goal_x,int goal_y){  // Astar路径规划 非阻塞版
        // 新任务（参数变化）或上次已完成 → 重新规划
        if(nb_state == NB_DONE || nb_sta_x != sta_x || nb_sta_y != sta_y ||
           nb_goal_x != goal_x || nb_goal_y != goal_y){
                nb_sta_x = sta_x;   nb_sta_y = sta_y;
                nb_goal_x = goal_x; nb_goal_y = goal_y;
                nb_state = NB_IDLE;
        }
        nb_debug_send();        // 与原阻塞循环相同的调试输出（注意删除）

        switch(nb_state){
        case NB_IDLE:
                if(!Find_path(sta_x, sta_y, goal_x, goal_y))
                        return false;   // 规划失败，保持 IDLE 下次重试
                if(path_length <= 2){// 与阻塞版一致：path_length<=2 时循环不执行直接返回 true
                        nb_state = NB_DONE; //（注意：起点终点相邻时不会移动，属阻塞版原有行为）
                        return true;
                }
                nb_j = 2;
                nb_old = 0;         // 注意：阻塞版 old 是 static 跨任务不清零，这里每次新任务归零，
                                    // 如需与阻塞版完全一致可删除本行
                nb_angle = -90*(path[nb_j].x - path[nb_j-2].x)/2;
                if(abs((int)(nb_angle - nb_old)) == 180)
                        nb_angle = nb_old;
                nb_state = NB_TURN;
                return false;

        case NB_TURN:           // 原地转向：等效原 pid_to_goal(current_x,current_y,angle)
                if(pid_to_v(current_x, current_y, nb_angle)){
                        nb_old = nb_angle;
                        if(nb_j+2 < path_length && (abs(path[nb_j+2].y - path[nb_j-2].y) == 4 || abs(path[nb_j+2].x -
path[nb_j-2].x) == 4)){ nb_j += 2;  // 直行段跳过一个节点（与阻塞版一致）
                        }
                        nb_state = NB_MOVE;
                }
                return false;

        case NB_MOVE:           // 走向当前节点
                if(pid_to_v(nodes[path[nb_j].x][path[nb_j].y].real_x , nodes[path[nb_j].x][path[nb_j].y].real_y ,
nb_angle)){ nb_j += 2; if(nb_j >= path_length){// 全部走完 nb_state = NB_DONE; return true;
                        }
                        nb_angle = -90*(path[nb_j].x - path[nb_j-2].x)/2;
                        if(abs((int)(nb_angle - nb_old)) == 180)
                                nb_angle = nb_old;
                        nb_state = NB_TURN;
                }
                return false;

        default:
                return false;
        }
}
*/
/*--------------------------------------------------------------------
 * main.c 中 STATE_NAV 需同步改为下面的写法（非阻塞版必须这样调用）：
 *--------------------------------------------------------------------*/
/*
                case STATE_NAV:{
                        switch(posit_state){
                        case 0: if(pid_to_goal_nb(200, 200, 0)){ posit_state = 1; } break;
                        case 1: if(pid_to_path_nb(0,0,2,0)){    posit_state = 2; } break;//路径规划
                        case 2: if(pid_to_path_nb(2,0,4,2)){    posit_state = 3; } break;
                        case 3: if(pid_to_path_nb(4,2,0,2)){    posit_state = 4; } break;
                        case 4: if(pid_to_path_nb(0,2,2,4)){    posit_state = 5; } break;
                        case 5: if(pid_to_path_nb(2,4,4,2)){    posit_state = 6; } break;
                        case 6: if(pid_to_path_nb(4,2,0,2)){    posit_state = 7; } break;
                        case 7: if(pid_to_path_nb(0,2,2,4)){    posit_state = 8; } break;
                        case 8: if(pid_to_path_nb(2,4,0,0)){    posit_state = 9; } break;
                        case 9: if(pid_to_goal_nb(15, -15, 0)){//最后一步定位单独占一个状态
                                        posit_state = 0;
                                        currentState = STATE_STOP;
                                } break;
                        }
                        break;
                }
*/
