/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 两轮自平衡小车（Jaxbalancecar）
  *
  *  本工程由 D:\blue\平衡车代码（标准库版）完整迁移而来，全部改为 HAL 库：
  *    pid.c     直立环(5ms) + 速度环(6ms) + 转向阻尼 + 倒地保护（原 main.c 的两个中断）
  *    motor.c   双电机方向 + PWM（原 Hardware/Motor.c）
  *    encode.c  左轮 EXTI 软件解码 + 右轮 TIM3 硬件编码器（原 Hardware/Encoder.c）
  *    tim.c     TIM2=PWM，TIM3=编码器，TIM4=1ms 控制时基（新增）
  *    main.c    初始化 + 标定 + OLED 显示
  *
  *  与原工程的差别只有两处：
  *    1) 原工程用 TIM4(5ms)+TIM1(2ms×3=6ms) 两个定时器，
  *       本工程用 TIM4 1ms 时基分频出 5ms 与 6ms（HAL 回调里做）
  *    2) 原工程用按键2启停，本工程没有按键 → 上电延时 3 秒后自动启动
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "mpu6050.h"
#include "motor.h"
#include "encode.h"
#include "pid.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DISPLAY_PERIOD_MS   100     /* OLED 刷新周期 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t  display_buf[32];
uint32_t last_disp_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Display_Update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ------------------------------------------------------------------
 * OLED 显示（与原工程一致：标题 + 两个环的 PID 参数 + 角度/状态）
 * ------------------------------------------------------------------ */
static void Display_Update(void)
{
	/* 第 1 行是标题，初始化时写一次即可，这里不重复刷 */
	sprintf((char *)display_buf, "Kp:%.2f Kd:%.2f   ", Balance_Kp, Balance_Kd);
	OLED_ShowString(0, 2, display_buf, 16);

	sprintf((char *)display_buf, "Kp:%.2f Ki:%.3f  ", Speed_Kp, Speed_Ki);
	OLED_ShowString(0, 4, display_buf, 16);

	sprintf((char *)display_buf, "Ang:%+05.1f S%d  ", Angle - Angle_Zero, Stop);
	OLED_ShowString(0, 6, display_buf, 16);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t start_tick;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

	/* ---------- 1、显示屏 ---------- */
	OLED_Init();
	OLED_Clear();
	OLED_ShowString(0, 0, (uint8_t *)"Jax Balance Car", 16);

	/* ---------- 2、电机（方向引脚 + PWM 启动） ---------- */
	Motor_Init();

	/* ---------- 3、编码器：左轮 EXTI(PA0/PA1)，右轮 TIM3(PA6/PA7) ---------- */
	Encoder_Init();
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	/* ---------- 4、MPU6050 ---------- */
	if (MPU_Init() != 0)
	{
		OLED_ShowString(0, 2, (uint8_t *)"MPU6050 FAIL ", 16);
		while (1) { HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin); HAL_Delay(200); }
	}
	OLED_ShowString(0, 2, (uint8_t *)"MPU6050 OK   ", 16);

	/* ---------- 5、标定（车必须静止、竖直，约 1 秒） ---------- */
	OLED_ShowString(0, 4, (uint8_t *)"Calibrating  ", 16);
	PID_Calibrate();
	OLED_ShowString(0, 4, (uint8_t *)"Calibrated   ", 16);

	/* ---------- 6、留时间把车扶正 ---------- */
	start_tick = HAL_GetTick();
	while ((HAL_GetTick() - start_tick) < AUTO_START_DELAY_MS)
	{
		HAL_Delay(10);
	}

	/* ---------- 7、启动控制时基（TIM4 1ms → 5ms 直立环 / 6ms 速度环） ---------- */
	Stop = 0;
	HAL_TIM_Base_Start_IT(&htim4);

	last_disp_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		uint32_t now = HAL_GetTick();

		/* 控制全部在 TIM4 中断里跑，主循环只负责显示 */
		if ((now - last_disp_tick) >= DISPLAY_PERIOD_MS)
		{
			last_disp_tick = now;
			Display_Update();
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
