
/*
 * HC05蓝牙模块驱动头文件
 * 适用于MSPM0G3507微控制器
 */

#ifndef __HC05_H
#define __HC05_H
#include "pid.h"
#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "string.h"
/* 定义接收缓冲区大小 */
#define BLERX_LEN_MAX    200

/* 蓝牙连接状态标志 */
extern uint8_t Bluetooth_ConnectFlag;  // 蓝牙连接状态 = 0没有手机连接   = 1有手机连接

/* 接收缓冲区和标志 */
extern uint8_t BLERX_BUFF[BLERX_LEN_MAX];
extern uint8_t BLERX_FLAG;
extern uint8_t BLERX_LEN;

/* 接收状态和数据缓冲区 */
extern uint8_t Is_Receiving;
extern uint16_t RxLine;
extern uint8_t DataBuff[200];
extern uint8_t RxBuffer[1];

/* 函数声明 */
void BLE_Send_Bit(uint8_t ch);
void BLE_send_String(uint8_t *str);
void Clear_BLERX_BUFF(void);
void Bluetooth_Init(void);
int fputc(int ch, FILE *stream);
uint8_t Get_Bluetooth_ConnectFlag(void);
int Receive_Bluetooth_Data(void);
void Send_Bluetooth_Data(char *dat);

/* 数据解析和PID调节相关函数 */
float Get_Data(void);
void USART_PID_Adjust(uint8_t Motor_n);

#endif /* __HC05_H */
