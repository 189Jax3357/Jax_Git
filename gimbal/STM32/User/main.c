#include "stm32f10x.h"                  // Device header
#include "Servo.h"
#include "Serial.h"

/**
  * 二维舵机云台 —— STM32F103C8T6 主控
  *
  * 接线：
  *   OpenMV P4(TX)  -> STM32 PA10(USART1_RX)
  *   OpenMV P5(RX)  -> STM32 PA9 (USART1_TX)
  *   GND  <-> GND（必须共地）
  *   1 号舵机信号   -> PA0 (TIM2_CH1)
  *   2 号舵机信号   -> PA1 (TIM2_CH2)
  *   舵机电源       -> 独立 5V（≥2A），地线与 STM32 共地
  *
  * 协议：0xFF | X | Y | 0xFE     X/Y 为 0~255，128 为中间位置
  */

/* ============ 方向配置（装反了就改这里） ============ */
#define AXIS_SWAP   0		/* 1 = 交换两路（PA0 用 Y，PA1 用 X） */
#define INVERT_X    0		/* 1 = 水平轴反向 */
#define INVERT_Y    0		/* 1 = 俯仰轴反向 */
/* ==================================================== */

int main(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	uint8_t x = 128;
	uint8_t y = 128;
	uint8_t sx, sy;

	/* ---------- PC13 指示灯 ---------- */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_13);			// 熄灭（低电平点亮）

	/* ---------- 外设初始化 ---------- */
	Servo_Init();								// PA0 / PA1 输出 50Hz PWM
	Servo_Center();								// 上电两轴回中
	Serial_Init();								// 接收 OpenMV 数据

	/* ---------- 主循环：收到一帧就更新两个舵机 ---------- */
	while (1)
	{
		if (Serial_GetFrame(&x, &y) == 1)
		{
#if AXIS_SWAP
			sx = y;
			sy = x;
#else
			sx = x;
			sy = y;
#endif
#if INVERT_X
			sx = (uint8_t)(255 - sx);
#endif
#if INVERT_Y
			sy = (uint8_t)(255 - sy);
#endif

			Servo_SetValue(0, sx);				// 1 号舵机（PA0）
			Servo_SetValue(1, sy);				// 2 号舵机（PA1）

			GPIO_ResetBits(GPIOC, GPIO_Pin_13);	// 收到数据时点灯（通信指示）
		}
	}
}
