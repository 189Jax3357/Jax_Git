#include "encode.h"

/* 右轮：读 TIMx->CNT 后清零，返回本周期脉冲增量（TIM3 硬件编码器，PA6/PA7） */
int read_speed(TIM_HandleTypeDef *htim)
{
	int temp = (short)__HAL_TIM_GET_COUNTER(htim);
	__HAL_TIM_SET_COUNTER(htim, 0);
	return temp;
}

/* ------------------------------------------------------------------
 * 左轮编码器：PA0 = A 相，PA1 = B 相
 * 由于 PA0/PA1 是 TIM2_CH1/CH2，而 TIM2 要被电机 PWM（PA2/PA3）占用，
 * 所以这里用外部中断做软件正交解码，4 倍频（A、B 相双沿都计数）。
 * ------------------------------------------------------------------ */

static volatile int32_t g_enc_left = 0;   /* 左编码器累计脉冲 */

void Encoder_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_AFIO_CLK_ENABLE();

	/* PA0 / PA1：上拉输入 + 上升沿和下降沿都触发中断 */
	GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* 优先级给高一点，避免高速时丢脉冲（数值越小优先级越高） */
	HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	g_enc_left = 0;
}

/* 读取左轮增量并清零（关中断保护，避免读写被打断） */
int read_speed_left(void)
{
	int32_t v;
	__disable_irq();
	v = g_enc_left;
	g_enc_left = 0;
	__enable_irq();
	return (int)v;
}

/* 外部中断服务函数（CubeMX 没生成 EXTI，所以在这里自己定义） */
void EXTI0_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void EXTI1_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

/* HAL 的中断回调：判断方向和计数 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_0)              /* A 相跳变 */
	{
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))
			g_enc_left++;                    /* A、B 同电平 -> 正转 */
		else
			g_enc_left--;                    /* 不同电平   -> 反转 */
	}
	else if (GPIO_Pin == GPIO_PIN_1)          /* B 相跳变 */
	{
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
			g_enc_left--;                    /* B、A 同电平 -> 反转 */
		else
			g_enc_left++;                    /* 不同电平   -> 正转 */
	}
}

/* ============================================================================
 *  兼容原工程（Hardware/Encoder.c）的接口
 *    左轮：PA0/PA1 EXTI 软件正交解码
 *    右轮：TIM3 硬件编码器
 * ========================================================================== */
extern TIM_HandleTypeDef htim3;

int16_t Encoder_Get_Left(void)
{
	return (int16_t)read_speed_left();
}

int16_t Encoder_Get_Right(void)
{
	return (int16_t)read_speed(&htim3);
}
