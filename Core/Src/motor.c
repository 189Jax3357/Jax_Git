/* ============================================================================
 *  电机驱动 —— 从 D:\blue\平衡车代码\Hardware\Motor.c 迁移到 HAL 库
 *
 *  硬件（与源工程一致）：
 *    左轮：方向 PA15 / PB3，PWM = TIM2_CH3 → PA2
 *    右轮：方向 PA4  / PA5，PWM = TIM2_CH4 → PA3
 *    速度范围：-100 ~ +100（对应原工程 PWM_SetCompare3/4 的 0~100）
 * ========================================================================== */
#include "motor.h"

int motor_abs(int p)
{
	return (p > 0) ? p : -p;
}

/* 把 -100~100 换算成 TIM2 的比较值（0 ~ PWM_MAX） */
static uint32_t speed_to_ccr(int8_t speed)
{
	int v = motor_abs((int)speed);
	if (v > 100) v = 100;
	return (uint32_t)((uint32_t)v * (uint32_t)PWM_MAX / 100u);
}

void Motor_Init(void)
{
	/* 方向引脚已在 MX_GPIO_Init() 里配置为推挽输出，
	   这里只把两个 PWM 通道启动起来，并让电机停转 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	Motor_SetSpeed_Left(0);
	Motor_SetSpeed_Right(0);
}

void Motor_SetSpeed_Left(int8_t Speed)
{
#if INVERT_LEFT
	Speed = -Speed;
#endif
	if (Speed > 0)                       /* 正转 */
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, speed_to_ccr(Speed));
	}
	else if (Speed < 0)                  /* 反转 */
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, speed_to_ccr((int8_t)(-Speed)));
	}
	else                                 /* 刹车 */
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	}
}

void Motor_SetSpeed_Right(int8_t Speed)
{
#if INVERT_RIGHT
	Speed = -Speed;
#endif
	if (Speed > 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed_to_ccr(Speed));
	}
	else if (Speed < 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed_to_ccr((int8_t)(-Speed)));
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	}
}

/* 兼容接口：直接给 ±PWM_MAX 的比较值 */
void Load(int moto1, int moto2)
{
	if (moto1 > PWM_MAX) moto1 = PWM_MAX;
	if (moto1 < PWM_MIN) moto1 = PWM_MIN;
	if (moto2 > PWM_MAX) moto2 = PWM_MAX;
	if (moto2 < PWM_MIN) moto2 = PWM_MIN;

	Motor_SetSpeed_Left((int8_t)(moto1 * 100 / PWM_MAX));
	Motor_SetSpeed_Right((int8_t)(moto2 * 100 / PWM_MAX));
}

void Limit(int *motoA, int *motoB)
{
	if (*motoA > PWM_MAX) *motoA = PWM_MAX;
	if (*motoA < PWM_MIN) *motoA = PWM_MIN;
	if (*motoB > PWM_MAX) *motoB = PWM_MAX;
	if (*motoB < PWM_MIN) *motoB = PWM_MIN;
}
