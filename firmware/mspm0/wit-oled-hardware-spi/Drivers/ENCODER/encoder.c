#include "encoder.h"
uint32_t gpio_interrup1,gpio_interrup2;
extern volatile int32_t Get_Encoder_countA,Get_Encoder_countB;
volatile uint32_t encoder_e1a_edges;
volatile uint32_t encoder_e1b_edges;
volatile uint32_t encoder_e2a_edges;
volatile uint32_t encoder_e2b_edges;
/*******************************************************
º¯Êý¹¦ÄÜ£ºÍâ²¿ÖÐ¶ÏÄ£Äâ±àÂëÆ÷ÐÅºÅ
Èë¿Úº¯Êý£ºÎÞ
·µ»Ø  Öµ£ºÎÞ
***********************************************************/
void GROUP1_IRQHandler(void)
{
	//»ñÈ¡ÖÐ¶ÏÐÅºÅ
    gpio_interrup1 = DL_GPIO_getEnabledInterruptStatus(
        ENCODERA_PORT, ENCODERA_E1A_PIN);
    gpio_interrup2 = DL_GPIO_getEnabledInterruptStatus(
        ENCODERB_PORT, ENCODERB_E2A_PIN);
    
    
	//encoderB
	if((gpio_interrup1 & ENCODERA_E1A_PIN)==ENCODERA_E1A_PIN)
	{
        encoder_e1a_edges++;
        Get_Encoder_countB++;
	}
	
	//encoderA
	if((gpio_interrup2 & ENCODERB_E2A_PIN)==ENCODERB_E2A_PIN)
	{
        encoder_e2a_edges++;
        Get_Encoder_countA++;
	}
	DL_GPIO_clearInterruptStatus(ENCODERA_PORT,ENCODERA_E1A_PIN|ENCODERA_E1B_PIN);
	DL_GPIO_clearInterruptStatus(ENCODERB_PORT,ENCODERB_E2A_PIN|ENCODERB_E2B_PIN);
}
