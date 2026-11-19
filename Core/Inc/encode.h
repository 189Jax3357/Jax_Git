#ifndef ENCODE_H
#define ENCODE_H

#include "stm32f1xx_hal.h"

/* 右轮：TIM3 硬件编码器计数（PA6/PA7） */
int read_speed(TIM_HandleTypeDef *htim);

/* 左轮：PA0/PA1 外部中断软件正交解码（不占用定时器） */
void Encoder_Init(void);
int  read_speed_left(void);

int16_t Encoder_Get_Left(void);
int16_t Encoder_Get_Right(void);

#endif
