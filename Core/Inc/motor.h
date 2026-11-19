#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"
#include "tim.h"

/* PWM 比较值满量程（TIM2 ARR = 7199） */
#define PWM_MAX   7200
#define PWM_MIN  (-PWM_MAX)

/* 方向修正：某个轮子转反了就把它改成 1 */
#define INVERT_LEFT    0
#define INVERT_RIGHT   0

int  motor_abs(int p);

/* 电机初始化：方向引脚 + 启动 PWM 通道（对应原工程 Motor_Init） */
void Motor_Init(void);

/* 设置速度，范围 -100 ~ 100（对应原工程 Motor_SetSpeed_Left / Right） */
void Motor_SetSpeed_Left(int8_t Speed);
void Motor_SetSpeed_Right(int8_t Speed);

/* 低层：直接给 ±7200 的比较值 */
void Load(int moto1, int moto2);
void Limit(int *motoA, int *motoB);

#endif
