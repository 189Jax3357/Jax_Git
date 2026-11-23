#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"

/* 通信协议（与 OpenMV 端一致）：
 *   0xFF | X | Y | 0xFE
 *   X、Y 为 0~255 的舵机目标值（128 为中间位置）
 */

void Serial_Init(void);                              /* USART1: PA9=TX, PA10=RX, 115200 */

uint8_t Serial_GetFrame(uint8_t *x, uint8_t *y);     /* 收到一帧新数据返回 1，并把 X/Y 取出 */

#endif
