/* ============================================================================
 *  平衡车控制 —— 从 D:\blue\平衡车代码（标准库版）完整迁移到 HAL 库
 *
 *  对应关系：
 *    原工程 TIM4_IRQHandler (5ms)  →  PID_Balance_Loop()
 *    原工程 TIM1_UP_IRQHandler (2ms，内部 3 分频 = 6ms) → PID_Speed_Loop()
 *
 *  触发方式：新增 TIM4 做 1ms 时基，在 HAL_TIM_PeriodElapsedCallback 里
 *            每 5 个 tick 跑直立环、每 6 个 tick 跑速度环。
 *            这样不需要 MPU6050 的 INT 引脚，也不需要额外定时器。
 * ========================================================================== */
#include "pid.h"
#include "encode.h"
#include "mpu6050.h"
#include "motor.h"
#include <math.h>

/* ---------- 传感器原始值 ---------- */
int16_t AX, AY, AZ, GX, GY, GZ;

/* ---------- 姿态 ---------- */
float Acc_Angle;                 /* 加速度计算的角度 */
float Gyro_Rate;                 /* 陀螺仪角速度 */
float Gyro_Filtered;             /* 低通后的角速度（观察用） */
float Angle;                     /* 互补滤波角度 */
float Gyro_Offset = 0.0f;        /* 陀螺仪零偏 */
float GZ_Offset   = 0.0f;        /* GZ 零偏 */
float Angle_Zero  = ANGLE_ZERO_VALUE;
float Angle_Limit = ANGLE_LIMIT_VALUE;

/* ---------- 环参数 ---------- */
float Balance_Kp = BALANCE_KP_VALUE, Balance_Kd = BALANCE_KD_VALUE;
float Speed_Kp   = SPEED_KP_VALUE,   Speed_Ki   = SPEED_KI_VALUE;
float Turn_Kp    = TURN_KP_VALUE;

/* ---------- 环变量 ---------- */
float Balance_Error = 0.0f, Balance_Out = 0.0f;

float Speed_Target = SPEED_TARGET_VALUE;
float Speed_Target_Ramp = 0.0f;
float Speed_Actual = 0.0f;
float Speed_Error = 0.0f, Speed_ErrorInt = 0.0f;
float Speed_ErrorLow = 0.0f, Speed_ErrorLow_Last = 0.0f;
float Speed_Out = 0.0f;

float Turn_Out = 0.0f;

/* ---------- 状态 ---------- */
uint8_t Stop = 0;                /* 0 = 运行（AUTO_START=1 时上电即运行） */
uint8_t Data_Ready = 0;

extern TIM_HandleTypeDef htim3;  /* 右轮编码器 */

/* ============================================================================
 *  标定：陀螺仪零偏（GY/GZ）+ 角度零点
 *  与源工程一致：静止采样 200 次取平均
 * ========================================================================== */
void PID_Calibrate(void)
{
	float Sum = 0.0f, Sum_Z = 0.0f;
	int i;

	for (i = 0; i < 200; i++)
	{
		MPU_Get_Gyroscope(&GX, &GY, &GZ);
		Sum   += (float)GY;
		Sum_Z += (float)GZ;
		HAL_Delay(5);
	}

	Gyro_Offset = Sum   / 200.0f;
	GZ_Offset   = Sum_Z / 200.0f;

	Angle = Angle_Zero;
	Gyro_Filtered = 0.0f;

	Speed_ErrorInt = 0.0f;
	Speed_ErrorLow = 0.0f;
	Speed_ErrorLow_Last = 0.0f;
	Speed_Target_Ramp = 0.0f;
	Balance_Out = 0.0f;
	Speed_Out = 0.0f;
	Turn_Out = 0.0f;
}

/* ============================================================================
 *  速度环（6ms 一次）—— 对应原工程 TIM1_UP_IRQHandler
 * ========================================================================== */
void PID_Speed_Loop(void)
{
	/* 实测速度 = 左右编码器增量平均，取负（与原工程一致） */
	Speed_Actual = -(Encoder_Get_Left() + Encoder_Get_Right()) / 2.0f;

	/* 软启动：目标值一步一步挪过去 */
	if      (Speed_Target - Speed_Target_Ramp >  SPEED_RAMP_STEP) Speed_Target_Ramp += SPEED_RAMP_STEP;
	else if (Speed_Target_Ramp - Speed_Target >  SPEED_RAMP_STEP) Speed_Target_Ramp -= SPEED_RAMP_STEP;
	else                                                          Speed_Target_Ramp  = Speed_Target;

	Speed_Error = Speed_Target_Ramp - Speed_Actual;

	/* 编码器误差一阶低通后再进积分，P 项仍用原始误差 */
	Speed_ErrorLow = 0.2f * Speed_Error + 0.8f * Speed_ErrorLow_Last;
	Speed_ErrorLow_Last = Speed_ErrorLow;

	Speed_ErrorInt += Speed_ErrorLow;
	if (Speed_ErrorInt >=  SPEED_INT_LIMIT) Speed_ErrorInt =  SPEED_INT_LIMIT;
	if (Speed_ErrorInt <= -SPEED_INT_LIMIT) Speed_ErrorInt = -SPEED_INT_LIMIT;

	/* 倾角偏太大就清积分 */
	if ((Angle - Angle_Zero) > SPEED_PROTECT_ANGLE || (Angle - Angle_Zero) < -SPEED_PROTECT_ANGLE)
	{
		Speed_ErrorInt = 0.0f;
	}

	Speed_Out = Speed_Kp * Speed_Error + Speed_Ki * Speed_ErrorInt;

	if (Speed_Out >=  SPEED_OUT_LIMIT) Speed_Out =  SPEED_OUT_LIMIT;
	if (Speed_Out <= -SPEED_OUT_LIMIT) Speed_Out = -SPEED_OUT_LIMIT;
}

/* ============================================================================
 *  直立环（5ms 一次）—— 对应原工程 TIM4_IRQHandler
 * ========================================================================== */
void PID_Balance_Loop(void)
{
	float Base, v_L, v_R;

	/* 1、读 MPU6050 原始数据 */
	MPU_Get_Accelerometer(&AX, &AY, &AZ);
	MPU_Get_Gyroscope(&GX, &GY, &GZ);

	/* 2、陀螺仪角速度（本车安装朝向使 GY 与 Angle 反号，取负对齐） */
	Gyro_Rate = (Gyro_Offset - (float)GY) / 16.4f;

	/* 3、角速度低通（仅观察用，D 项直接用原始值） */
	Gyro_Filtered = 0.9f * Gyro_Filtered + 0.1f * Gyro_Rate;

	/* 4、互补滤波算角度 */
	Acc_Angle = atan2f((float)AX, (float)AZ) * 180.0f / 3.14159f;
	Angle = 0.9f * (Angle + Gyro_Rate * 0.005f) + 0.1f * Acc_Angle;

	/* 5、直立环 PD */
	Balance_Error = 0.0f - (Angle - Angle_Zero);
	Balance_Out = Balance_Kp * Balance_Error - Balance_Kd * Gyro_Rate;

	if (Balance_Out >  BALANCE_OUT_LIMIT) Balance_Out =  BALANCE_OUT_LIMIT;
	if (Balance_Out < -BALANCE_OUT_LIMIT) Balance_Out = -BALANCE_OUT_LIMIT;

	/* 6、转向阻尼（陀螺仪 Z 轴） */
	Turn_Out = Turn_Kp * (((float)GZ - GZ_Offset) / 16.4f);
	if (Turn_Out >  TURN_OUT_LIMIT) Turn_Out =  TURN_OUT_LIMIT;
	if (Turn_Out < -TURN_OUT_LIMIT) Turn_Out = -TURN_OUT_LIMIT;

	/* 7、倒地保护 + 左右轮混控输出 */
	if ((Angle - Angle_Zero) > Angle_Limit || (Angle - Angle_Zero) < -Angle_Limit)
	{
		Motor_SetSpeed_Left(0);
		Motor_SetSpeed_Right(0);
	}
	else if (Stop == 0)
	{
		Base = Balance_Out + Speed_Out;
		v_L  = Base - Turn_Out;
		v_R  = Base + Turn_Out;

		if (v_L >  100.0f) v_L =  100.0f;
		if (v_L < -100.0f) v_L = -100.0f;
		if (v_R >  100.0f) v_R =  100.0f;
		if (v_R < -100.0f) v_R = -100.0f;

		/* 与原工程一致：右轮取负（镜像安装） */
		Motor_SetSpeed_Left((int8_t)v_L);
		Motor_SetSpeed_Right((int8_t)(-v_R));
	}
	else
	{
		Motor_SetSpeed_Left(0);
		Motor_SetSpeed_Right(0);
	}

	Data_Ready = 1;
}

/* ============================================================================
 *  TIM4 更新中断（1ms）—— 分频出 5ms 直立环 与 6ms 速度环
 *  原工程是 TIM4(5ms)=直立环 + TIM1(2ms×3=6ms)=速度环，这里合并成一个时基
 * ========================================================================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t tick5 = 0, tick6 = 0;

	if (htim->Instance != TIM4)
	{
		return;
	}

	/* 直立环：5ms */
	if (++tick5 >= 5)
	{
		tick5 = 0;
		PID_Balance_Loop();
	}

	/* 速度环：6ms */
	if (++tick6 >= 6)
	{
		tick6 = 0;
		PID_Speed_Loop();
	}
}
