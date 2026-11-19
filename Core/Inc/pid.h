#ifndef PID_H
#define PID_H

#include "stm32f1xx_hal.h"

/* ============================================================================
 *  参数 —— 全部来自 D:\blue\平衡车代码\User\main.c（标准库版），数值未改动
 * ========================================================================== */
#define BALANCE_KP_VALUE     2.31f      /* 直立环 P */
#define BALANCE_KD_VALUE     0.41f      /* 直立环 D */
#define SPEED_KP_VALUE       5.4f       /* 速度环 P */
#define SPEED_KI_VALUE       0.027f     /* 速度环 I */
#define TURN_KP_VALUE        0.4f       /* 转向阻尼 */
#define ANGLE_ZERO_VALUE     (-2.81f)   /* 竖直时的角度零点 */
#define ANGLE_LIMIT_VALUE    45.0f      /* 倒地保护角 */
#define SPEED_TARGET_VALUE   0.0f       /* 目标速度（0 = 原地平衡） */
#define SPEED_INT_LIMIT      300.0f     /* 速度环积分限幅 */
#define SPEED_OUT_LIMIT      100.0f     /* 速度环输出限幅 */
#define BALANCE_OUT_LIMIT    100.0f     /* 直立环输出限幅 */
#define TURN_OUT_LIMIT       50.0f      /* 转向输出限幅 */
#define SPEED_PROTECT_ANGLE  30.0f      /* 倾角超过它清速度环积分 */
#define SPEED_RAMP_STEP      0.1f       /* 软启动每步最大变化 */

/* 原工程用按键2启动；本工程没有按键，上电延时后自动启动。
   想改回"上电停住"，把这里改成 0，并把 Stop 初值设为 1 */
#define AUTO_START           1
#define AUTO_START_DELAY_MS  3000

/* ============================================================================
 *  传感器与控制量（供 OLED 显示）
 * ========================================================================== */
extern int16_t AX, AY, AZ, GX, GY, GZ;

extern float Acc_Angle;         /* 加速度计算的角度 */
extern float Gyro_Rate;         /* 陀螺仪角速度（度/秒） */
extern float Gyro_Filtered;     /* 角速度低通值 */
extern float Angle;             /* 互补滤波后的角度 */
extern float Gyro_Offset;       /* 陀螺仪零偏 */
extern float GZ_Offset;         /* GZ 零偏 */
extern float Angle_Zero;        /* 角度零点 */
extern float Angle_Limit;       /* 倒地保护角 */

extern float Balance_Kp, Balance_Kd;
extern float Speed_Kp, Speed_Ki;
extern float Turn_Kp;

extern float Balance_Error, Balance_Out;
extern float Speed_Target, Speed_Target_Ramp, Speed_Actual;
extern float Speed_Error, Speed_ErrorInt, Speed_ErrorLow, Speed_ErrorLow_Last, Speed_Out;
extern float Turn_Out;

extern uint8_t Stop;            /* 0 = 运行，1 = 停机 */
extern uint8_t Data_Ready;      /* 每完成一次直立环置 1，供显示用 */

/* 陀螺仪零偏 + 角度零点标定（车要静止、竖直） */
void PID_Calibrate(void);

/* 5ms：直立环（互补滤波 + PD + 转向阻尼 + 倒地保护 + 电机输出） */
void PID_Balance_Loop(void);

/* 6ms：速度环（PI，软启动，误差低通，积分限幅） */
void PID_Speed_Loop(void);

#endif
