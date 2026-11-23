#include "Serial.h"

#define FRAME_HEAD	0xFF
#define FRAME_TAIL	0xFE

/* 接收状态机：
 *   0: 等帧头 0xFF
 *   1: 收第 1 个数据字节（X）
 *   2: 收第 2 个数据字节（Y）
 *   3: 等帧尾 0xFE
 */
static volatile uint8_t s_state = 0;
static volatile uint8_t s_x = 128;
static volatile uint8_t s_y = 128;
static volatile uint8_t s_new = 0;

/**
  * 函    数：串口1初始化（PA9 = TX，PA10 = RX，115200 8N1）
  * 说    明：OpenMV 的 P4(TX) 接 PA10(RX)，OpenMV 的 P5(RX) 接 PA9(TX)，
  *           两边必须共地
  */
void Serial_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

	/* PA9 复用推挽输出（TX） */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* PA10 上拉输入（RX） */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 参数：115200，8 位数据，无校验，1 位停止，收发模式，无流控 */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStructure);

	/* 开启接收中断 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART1, ENABLE);
}

/**
  * 函    数：串口1接收中断
  * 说    明：逐字节喂给状态机，凑齐 0xFF X Y 0xFE 才算一帧
  */
void USART1_IRQHandler(void)
{
	uint8_t b;

	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		b = (uint8_t)USART_ReceiveData(USART1);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);

		switch (s_state)
		{
			case 0:
				if (b == FRAME_HEAD)
				{
					s_state = 1;
				}
				break;

			case 1:
				s_x = b;
				s_state = 2;
				break;

			case 2:
				s_y = b;
				s_state = 3;
				break;

			case 3:
				if (b == FRAME_TAIL)
				{
					s_new = 1;			/* 一帧完整，交给主循环 */
					s_state = 0;
				}
				else if (b == FRAME_HEAD)
				{
					s_state = 1;		/* 帧尾错位，但这一字节可能是新的帧头 */
				}
				else
				{
					s_state = 0;		/* 丢弃，重新找帧头 */
				}
				break;

			default:
				s_state = 0;
				break;
		}
	}
}

/**
  * 函    数：取一帧数据
  * 参    数：x、y 用于返回两个舵机目标值
  * 返 回 值：1 = 有新的完整帧；0 = 没有新数据
  */
uint8_t Serial_GetFrame(uint8_t *x, uint8_t *y)
{
	if (s_new == 0)
	{
		return 0;
	}

	*x = s_x;
	*y = s_y;
	s_new = 0;
	return 1;
}
