#include "Servo.h"

/**
  * 函    数：舵机云台初始化（TIM2_CH1 -> PA0，TIM2_CH2 -> PA1）
  * 说    明：舵机要求 50Hz（周期 20ms），高电平 0.5~2.5ms 对应 0~180°
  *           TIM2 时钟 72MHz，预分频 72-1 得到 1MHz 计数（1 个计数 = 1us）
  *           周期 20000-1，即 20000us = 20ms
  *           于是比较值 CCR 直接就是脉宽的微秒数（500~2500）
  */
void Servo_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* 开启时钟：GPIOA 在 APB2，TIM2 在 APB1 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* PA0 / PA1 配置为复用推挽输出（由定时器接管） */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 时基单元：1MHz 计数，20ms 周期 -> 50Hz */
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

	/* 输出比较：PWM1 模式，高电平有效 */
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = SERVO_MID_US;

	TIM_OC1Init(TIM2, &TIM_OCInitStructure);		/* CH1 -> PA0 */
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);		/* CH2 -> PA1 */

	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM2, ENABLE);

	TIM_Cmd(TIM2, ENABLE);
}

/**
  * 函    数：设置某个舵机的脉宽
  * 参    数：ch 0 = 水平舵机(PA0)，1 = 俯仰舵机(PA1)
  *           us 脉宽（微秒），自动限制在 SERVO_MIN_US ~ SERVO_MAX_US
  */
void Servo_SetPulse(uint8_t ch, uint16_t us)
{
	if (us < SERVO_MIN_US) us = SERVO_MIN_US;
	if (us > SERVO_MAX_US) us = SERVO_MAX_US;

	if (ch == 0)
	{
		TIM_SetCompare1(TIM2, us);
	}
	else
	{
		TIM_SetCompare2(TIM2, us);
	}
}

/**
  * 函    数：用 0~255 的值控制舵机（与 OpenMV 发来的字节对应）
  * 换    算：us = 500 + val * 2000 / 255
  *           val=0   -> 500us
  *           val=128 -> 约 1504us（中位）
  *           val=255 -> 2500us
  */
void Servo_SetValue(uint8_t ch, uint8_t val)
{
	uint16_t us;

	us = (uint16_t)(SERVO_MIN_US + ((uint32_t)val * (SERVO_MAX_US - SERVO_MIN_US)) / 255);

	Servo_SetPulse(ch, us);
}

/**
  * 函    数：两个舵机回中
  */
void Servo_Center(void)
{
	Servo_SetPulse(0, SERVO_MID_US);
	Servo_SetPulse(1, SERVO_MID_US);
}
