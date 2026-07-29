#ifndef __GRAY_H
#define __GRAY_H
#include "ti_msp_dl_config.h"
#define SysTickMAX_COUNT 0xFFFFFF
void Huidu_Init(void);
void Huidu_GetNum_Start(void);
char Huidu_GetNum(void);
uint32_t Systick_getTick(void);
void delay_us(uint32_t us);
#define SysTickFre 80000000



#define SysTick_US(x)  ((SysTickFre/1000000U)*(uint32_t)(x))
#endif