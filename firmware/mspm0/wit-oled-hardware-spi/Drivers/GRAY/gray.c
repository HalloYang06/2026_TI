#include "gray.h"
#define HUIDU_ADDRESS 0x9E
#define HUIDU_NUM_ADDRESS 0xDD
uint32_t Systick_getTick(void)
{
	return (SysTick->VAL);
}
void delay_us(uint32_t us)
{
	if( us > SysTickMAX_COUNT/(SysTickFre/1000000) ) us = SysTickMAX_COUNT/(SysTickFre/1000000);
	
	us = us*(SysTickFre/1000000); 
	

	uint32_t runningtime = 0;
	

	uint32_t InserTick = Systick_getTick();
	

	uint32_t tick = 0;
	
	uint8_t countflag = 0;

	while(1)
	{
		tick = Systick_getTick();
		
		if( tick > InserTick ) countflag = 1;
		
		if( countflag ) runningtime = InserTick + SysTickMAX_COUNT - tick;
		else runningtime = InserTick - tick;
		
		if( runningtime>=us ) break;
	}

}
void Huidu_W_SCL(uint8_t Bitvalue)
{
    if(Bitvalue==1){
        DL_GPIO_setPins(GPIO_IIC_PORT, GPIO_IIC_PIN_SCL_PIN);
    }else{
        DL_GPIO_clearPins(GPIO_IIC_PORT, GPIO_IIC_PIN_SCL_PIN);
    }
    delay_us(10);
    
}
void Huidu_W_SDA(uint8_t Bitvalue)
{
    if(Bitvalue==1){
        DL_GPIO_setPins(GPIO_IIC_PORT, GPIO_IIC_PIN_SDA_PIN);
    }else{
        DL_GPIO_clearPins(GPIO_IIC_PORT, GPIO_IIC_PIN_SDA_PIN);
    }
    delay_us(10);

}
uint8_t Huidu_R_SDA(void)
{
    uint8_t Bitvalue;
    Bitvalue=DL_GPIO_readPins(GPIO_IIC_PORT,GPIO_IIC_PIN_SDA_PIN);
    delay_us(10);
    return Bitvalue;
}

void Huidu_Init(void)
{
    Huidu_W_SCL(1);
    Huidu_W_SDA(1);
}
void Huidu_Start(void)
{
    Huidu_W_SCL(1);
    Huidu_W_SDA(1);
    Huidu_W_SDA(0);
    Huidu_W_SCL(0);
}
void Huidu_Stop(void)
{
    Huidu_W_SCL(0);
    Huidu_W_SDA(0);
    Huidu_W_SCL(1);
    Huidu_W_SDA(1);
}
void Huidu_SendByte(uint8_t Byte)
{
    uint8_t i=0;
    for(i=0;i<8;i++)
    {
        Huidu_W_SDA(Byte &(0x80>>i));
        Huidu_W_SCL(1);
        Huidu_W_SCL(0);
    }
}
uint8_t Huidu_ReceiveByte(void)
{
    uint8_t Byte=0x00;
    Huidu_W_SDA(1);
    uint8_t i=0;
    for(i=0;i<8;i++)
    {
        Huidu_W_SCL(1);
        if(Huidu_R_SDA()==1)
        Byte|=(0x80>>i);
        Huidu_W_SCL(0);
    }
    return Byte;
}
uint8_t Huidu_ReceiveAck(void)
{
	uint8_t AckBit;
	Huidu_W_SDA(1);
	Huidu_W_SCL(1);
	AckBit = Huidu_R_SDA();
	Huidu_W_SCL(0);
	return AckBit;
}
void Huidu_SendAck(uint8_t AckBit)
{
Huidu_W_SDA(AckBit);
Huidu_W_SCL(1);
Huidu_W_SCL(0);
}
void Huidu_GetNum_Start(void)
{
	Huidu_Start();
	Huidu_SendByte(HUIDU_ADDRESS);
	Huidu_ReceiveAck();
	Huidu_SendByte(HUIDU_NUM_ADDRESS);
	Huidu_ReceiveAck();
	Huidu_Stop();
}
char Huidu_GetNum(void)
{
	char GetNum;
	Huidu_Start();
	Huidu_SendByte(HUIDU_ADDRESS | 0x9F);
	Huidu_ReceiveAck();
	GetNum = Huidu_ReceiveByte();
	Huidu_SendAck(1);
   Huidu_Stop();
return GetNum;
}