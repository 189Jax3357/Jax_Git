#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"

/* 舵机脉宽范围（us）：SG90/MG996R 通用 500~2500，中位 1500 */
#define SERVO_MIN_US    500
#define SERVO_MAX_US    2500
#define SERVO_MID_US    1500

/* 初始化：PA0 -> TIM2_CH1，PA1 -> TIM2_CH2，50Hz */
void Servo_Init(void);

/* 直接给脉宽（us），自动限幅到 500~2500 */
void Servo_SetPulse(uint8_t ch, uint16_t us);

/* 给 0~255 的值，线性映射到 500~2500us：0->500us, 128->约1500us, 255->2500us */
void Servo_SetValue(uint8_t ch, uint8_t val);

/* 两个舵机回中位 */
void Servo_Center(void);

#endif
