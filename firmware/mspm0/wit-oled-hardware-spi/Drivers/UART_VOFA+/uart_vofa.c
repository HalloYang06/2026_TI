/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 文档网站：wiki.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 嘉立创社区问答：https://www.jlc-bbs.com/lckfb
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 */

/*
 * HC05蓝牙模块驱动源文件
 * 适用于MSPM0G3507微控制器
 */

#include "stdio.h"
#include "uart_vofa.h"

/*定义PID结构体变量*/


/* 全局变量定义 */
uint8_t Bluetooth_ConnectFlag = 0;  // 蓝牙连接状态 = 0没有手机连接   = 1有手机连接
uint8_t BLERX_BUFF[BLERX_LEN_MAX];
uint8_t BLERX_FLAG = 0;
uint8_t BLERX_LEN = 0;

uint8_t Is_Receiving = 0;
uint16_t RxLine = 0;        // 指令长度
uint8_t DataBuff[200];      // 指令内容
uint8_t RxBuffer[1];        // 临时接收缓冲区
#include "stdio.h"

//重定向fputc函数
int fputc(int ch, FILE *stream)
{
    while( DL_UART_isBusy(UART_0_INST) == true );
    DL_UART_Main_transmitData(UART_0_INST, ch);
    return ch;
}

//重定向fputs函数
int fputs(const char* restrict s, FILE* restrict stream) {

    uint16_t char_len=0;
    while(*s!=0)
    {
        while( DL_UART_isBusy(UART_0_INST) == true );
        DL_UART_Main_transmitData(UART_0_INST, *s++);
        char_len++;
    }
    return char_len;
}
int puts(const char* _ptr)
{
 return 0;
}

#if !defined(__MICROLIB)
//不使用微库的话就需要添加下面的函数
#if (__ARMCLIB_VERSION <= 6000000)
//如果编译器是AC5  就定义下面这个结构体
struct __FILE
{
        int handle;
};
FILE __stdout;
#endif
//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
        x = x;
}
#endif
/******************************************************************
 * 函 数 名 称：BLE_Send_Bit
 * 函 数 说 明：向蓝牙发送单个字符
 * 函 数 形 参：ch=ASCII字符
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void BLE_Send_Bit(uint8_t ch)
{
    //当串口1忙的时候等待，不忙的时候再发送传进来的字符
    while( DL_UART_isBusy(UART_0_INST) == true );
    //发送单个字符
    DL_UART_Main_transmitData(UART_0_INST, ch);
}

/******************************************************************
 * 函 数 名 称：BLE_send_String
 * 函 数 说 明：向蓝牙发送字符串
 * 函 数 形 参：str=发送的字符串
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void BLE_send_String(uint8_t *str)
{
    while( str && *str ) // 地址为空或者值为空跳出
    {
        BLE_Send_Bit(*str++);
    }
}

/******************************************************************
 * 函 数 名 称：Clear_BLERX_BUFF
 * 函 数 说 明：清除串口接收的数据
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
void Clear_BLERX_BUFF(void)
{
    BLERX_LEN = 0;
    BLERX_FLAG = 0;
}

/******************************************************************
 * 函 数 名 称：Bluetooth_Init
 * 函 数 说 明：蓝牙初始化
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：默认波特率为9600
******************************************************************/
void Bluetooth_Init(void)
{
    //清除串口中断标志
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    //使能串口中断
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    #if DEBUG
         //在调试时，通过AT命令已经设置好模式
    #endif
}

/******************************************************************
 * 函 数 名 称：Get_Bluetooth_ConnectFlag
 * 函 数 说 明：获取手机连接状态
 * 函 数 形 参：无
 * 函 数 返 回：返回1=已连接                返回0=未连接
 * 作       者：LC
 * 备       注：使用该函数前，必须先调用 Bluetooth_Mode 函数
******************************************************************/
uint8_t Get_Bluetooth_ConnectFlag(void)
{
    return Bluetooth_ConnectFlag;
}

/******************************************************************
 * 函 数 名 称：Receive_Bluetooth_Data
 * 函 数 说 明：接收蓝牙数据
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
******************************************************************/
int Receive_Bluetooth_Data(void)
{
    if( BLERX_FLAG == 1 )//接收到蓝牙数据
    {
        return  1;
    }
    else
    {
        return 0;
    }
}

/******************************************************************
 * 函 数 名 称：Send_Bluetooth_Data
 * 函 数 说 明：向蓝牙模块发送数据
 * 函 数 形 参：dat=要发送的字符串
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：（如果手机连接了蓝牙，就是向手机发送数据）
******************************************************************/
void Send_Bluetooth_Data(char *dat)
{
    //发送数据
    BLE_send_String((uint8_t*)dat);
}

/*
 * 解析出DataBuff中的数据
 * 返回解析得到的数据
 */
float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // 记录数据位开始的地方
    uint8_t data_End_Num = 0;   // 记录数据位结束的地方
    uint8_t data_Num = 0;       // 记录数据位数
    uint8_t minus_Flag = 0;     // 判断是不是负数
    float data_return = 0;      // 解析得到的数据
    
    for(uint8_t i=0; i<200; i++) // 查找等号和感叹号的位置
    {
        if(DataBuff[i] == '=') data_Start_Num = i + 1; // +1是直接定位到数据起始位
        if(DataBuff[i] == '!')
        {
            data_End_Num = i - 1;
            break;
        }
    }
    
    if(DataBuff[data_Start_Num] == '-') // 如果是负数
    {
        data_Start_Num += 1;  // 后移一位到数据位
        minus_Flag = 1;       // 负数flag
    }
    
    data_Num = data_End_Num - data_Start_Num + 1;
    
    // 根据数据位数解析浮点数
    if(data_Num == 4) // 数据共4位
    {
        data_return = (DataBuff[data_Start_Num]-48)  + (DataBuff[data_Start_Num+2]-48)*0.1f +
                      (DataBuff[data_Start_Num+3]-48)*0.01f;
    }
    else if(data_Num == 5) // 数据共5位
    {
        data_return = (DataBuff[data_Start_Num]-48)*10 + (DataBuff[data_Start_Num+1]-48) + 
                      (DataBuff[data_Start_Num+3]-48)*0.1f + (DataBuff[data_Start_Num+4]-48)*0.01f;
    }
    else if(data_Num == 6) // 数据共6位
    {
        data_return = (DataBuff[data_Start_Num]-48)*100 + (DataBuff[data_Start_Num+1]-48)*10 + 
                      (DataBuff[data_Start_Num+2]-48) + (DataBuff[data_Start_Num+4]-48)*0.1f + 
                      (DataBuff[data_Start_Num+5]-48)*0.01f;
    }
    else if(data_Num == 7) // 数据共7位
    {
        data_return = (DataBuff[data_Start_Num]-48)*1000 + (DataBuff[data_Start_Num+1]-48)*100 + 
                      (DataBuff[data_Start_Num+2]-48)*10 + (DataBuff[data_Start_Num+3]-48) +
                      (DataBuff[data_Start_Num+5]-48)*0.1f + (DataBuff[data_Start_Num+6]-48)*0.01f;
    }
    else if(data_Num == 8) // 数据共8位
    {
        data_return = (DataBuff[data_Start_Num]-48)*10000 + (DataBuff[data_Start_Num+1]-48)*1000 + 
                      (DataBuff[data_Start_Num+2]-48)*100 + (DataBuff[data_Start_Num+3]-48)*10 + 
                      (DataBuff[data_Start_Num+4]-48) + (DataBuff[data_Start_Num+6]-48)*0.1f + 
                      (DataBuff[data_Start_Num+7]-48)*0.01f;
    }
    
    if(minus_Flag == 1)  data_return = -data_return;
    return data_return;
}
 
/*
 * 根据串口信息进行PID调参
 */
void USART_PID_Adjust(uint8_t Motor_n)
{
    float data_Get = Get_Data(); // 存放接收到的数据
    
    // OLED显示接收到的数据
    // OLED_ShowNum(0, 32, data_Get, 5, OLED_8X16);
    // OLED_ShowString(0, 48, (char*)DataBuff, OLED_8X16);
    // OLED_Update();
    
    // 根据接收到的指令调整PID参数
    if(DataBuff[0] == 'K' && DataBuff[1] == '1'){ // 右轮P
        RIGHT.Kp = data_Get;
        PID_Update(&RIGHT);
    }
    if(DataBuff[0] == 'K' && DataBuff[1] == '2'){ // 右轮I
        RIGHT.Ki = data_Get;
        PID_Update(&RIGHT);
    }
    if(DataBuff[0] == 'K' && DataBuff[1] == '3'){ // 右轮D
        RIGHT.Kd = data_Get;
        PID_Update(&RIGHT);
    }
    if(DataBuff[0] == 'R' && DataBuff[1] == 't'){ // 右轮目标
        RIGHT.Target = data_Get;
        PID_Update(&RIGHT);
    }
    if(DataBuff[0] == 'P' && DataBuff[1] == '2'){ // 左轮P
        LEFT.Kp = data_Get;
        PID_Update(&LEFT);
    }
    if(DataBuff[0] == 'I' && DataBuff[1] == '2'){ // 左轮I
        LEFT.Ki = data_Get;
        PID_Update(&LEFT);
    }
    if(DataBuff[0] == 'D' && DataBuff[1] == '2'){ // 左轮D
        LEFT.Kd = data_Get;
        PID_Update(&LEFT);
    }
    if(DataBuff[0] == 'L' && DataBuff[1] == 't'){ // 左轮目标
        LEFT.Target = data_Get;
        PID_Update(&LEFT);
    }
    if(DataBuff[0] == 'P' && DataBuff[1] == '1'){ // 角P
        ANGLE.Kp = data_Get;
        PID_Update(&ANGLE);
    }
    if(DataBuff[0] == 'I' && DataBuff[1] == '1'){ // 角I
        ANGLE.Ki = data_Get;
        PID_Update(&ANGLE);
    }
    if(DataBuff[0] == 'D' && DataBuff[1] == '1'){ // 角D
        ANGLE.Kd = data_Get;
        PID_Update(&ANGLE);
    }
    if(DataBuff[0] == 'O' && DataBuff[1] == 't'){ // 角目标
        ANGLE.Target = data_Get;
        PID_Update(&ANGLE);
    }
}

// 串口的中断服务函数
void UART_0_INST_IRQHandler(void)
{
    //如果产生了串口中断
    switch( DL_UART_getPendingInterrupt(UART_0_INST) )
    {
        case DL_UART_IIDX_RX://如果是接收中断

            // 读取接收到的数据
            RxBuffer[0] = DL_UART_Main_receiveData(UART_0_INST);

            if (RxLine >= sizeof(DataBuff))
            {
                /* 长时间收不到结束符时丢弃本帧，避免越界破坏内存。 */
                memset(DataBuff, 0, sizeof(DataBuff));
                RxLine = 0;
                Is_Receiving = 0;
                break;
            }

            Is_Receiving = 1;
            DataBuff[RxLine++] = RxBuffer[0];

            if(RxBuffer[0] == 0x21)           //接收结束标志位！
            {
                USART_PID_Adjust(1);         //数据解析和参数赋值函数
                memset(DataBuff, 0, sizeof(DataBuff));  //清空缓存数组
                // OLED_ShowNum(0, 16, RxLine, 3, OLED_8X16);
                // OLED_Update();
                RxLine = 0;                  //清空接收长度
                Is_Receiving = 0;
            }
            
            // 同时更新原蓝牙接收缓冲区
            if (BLERX_LEN < BLERX_LEN_MAX - 1U) // 保留一个字符的空间用于'\0'
            {
                BLERX_BUFF[BLERX_LEN++] = RxBuffer[0]; // 接收数据
            }
            else
            {
                // 缓冲区已满，不保存数据
            }
            
            BLERX_BUFF[BLERX_LEN] = '\0';  // 确保字符串正确结束
            BLERX_FLAG = 1;  // 设置接收完成标志位

            break;

        default://其他的串口中断
            break;
    }
}
