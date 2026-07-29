#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "clock.h"
#include "mpu6050.h"
#include "bno08x_uart_rvc.h"
#include "wit.h"
#include "vl53l0x.h"
#include "lsm6dsv16x.h"
// 全局变量声明（放在函数外部）
uint8_t init_count = 0;          // 初始数据计数
float yaw_offset = 0.0f;         // yaw角偏移量
float initial_yaw_sum = 0.0f;    // 初始yaw角总和
uint8_t enable_group1_irq = 0;
float raw_yaw=0;
void Interrupt_Init(void)
{
    if(enable_group1_irq)
    {
        NVIC_EnableIRQ(1);
    }
}

void SysTick_Handler(void)
{
    tick_ms++;
}

#if defined UART_BNO08X_INST_IRQHandler
void UART_BNO08X_INST_IRQHandler(void)
{
    uint8_t checkSum = 0;
    extern uint8_t bno08x_dmaBuffer[19];

    DL_DMA_disableChannel(DMA, DMA_BNO08X_CHAN_ID);
    uint8_t rxSize = 18 - DL_DMA_getTransferSize(DMA, DMA_BNO08X_CHAN_ID);

    if(DL_UART_isRXFIFOEmpty(UART_BNO08X_INST) == false)
        bno08x_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_BNO08X_INST);

    for(int i=2; i<=14; i++)
        checkSum += bno08x_dmaBuffer[i];

    if((rxSize == 19) && (bno08x_dmaBuffer[0] == 0xAA) && (bno08x_dmaBuffer[1] == 0xAA) && (checkSum == bno08x_dmaBuffer[18]))
    {
        bno08x_data.index = bno08x_dmaBuffer[2];
        bno08x_data.yaw = (int16_t)((bno08x_dmaBuffer[4]<<8)|bno08x_dmaBuffer[3]) / 100.0;
        bno08x_data.pitch = (int16_t)((bno08x_dmaBuffer[6]<<8)|bno08x_dmaBuffer[5]) / 100.0;
        bno08x_data.roll = (int16_t)((bno08x_dmaBuffer[8]<<8)|bno08x_dmaBuffer[7]) / 100.0;
        bno08x_data.ax = (bno08x_dmaBuffer[10]<<8)|bno08x_dmaBuffer[9];
        bno08x_data.ay = (bno08x_dmaBuffer[12]<<8)|bno08x_dmaBuffer[11];
        bno08x_data.az = (bno08x_dmaBuffer[14]<<8)|bno08x_dmaBuffer[13];
    }
    
    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_BNO08X_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_BNO08X_CHAN_ID, (uint32_t) &bno08x_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_BNO08X_CHAN_ID, 18);
    DL_DMA_enableChannel(DMA, DMA_BNO08X_CHAN_ID);
}
#endif

#if defined UART_WIT_INST_IRQHandler
void UART_WIT_INST_IRQHandler(void)
{
    uint8_t checkSum;
    uint8_t offset = 0;
    uint8_t hasRxData = 0;
    extern uint8_t wit_dmaBuffer[33];

    /*
     * 读取 IIDX 会确认并清除当前中断。原代码没有读取挂起源，RX timeout
     * 会反复进入中断，主循环因而得不到运行时间。
     */
    if (DL_UART_Main_getPendingInterrupt(UART_WIT_INST) !=
        DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR)
    {
        return;
    }

    DL_DMA_disableChannel(DMA, DMA_WIT_CHAN_ID);
    uint8_t rxSize = 32 - DL_DMA_getTransferSize(DMA, DMA_WIT_CHAN_ID);

    if ((DL_UART_isRXFIFOEmpty(UART_WIT_INST) == false) &&
        (rxSize < sizeof(wit_dmaBuffer)))
    {
        wit_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_WIT_INST);
    }

    /*
     * On this device an idle RX-timeout can report DMASZ as zero even though
     * DMA did not write a byte.  The buffer is cleared before each transfer,
     * so reject an all-zero block instead of reporting 32 phantom bytes.
     * A WIT frame always contains the non-zero 0x55 header and type byte.
     */
    for (uint8_t i = 0; i < rxSize; i++)
    {
        if (wit_dmaBuffer[i] != 0U)
        {
            hasRxData = 1U;
            break;
        }
    }
    if (hasRxData == 0U)
    {
        rxSize = 0U;
    }
    wit_rx_byte_count += rxSize;

    /*
     * 不假定 DMA 缓冲区正好从 0x55 开始。逐字节寻找合法的 11 字节
     * WIT 数据帧，线路上出现丢字节时也能重新同步。
     */
    while ((uint8_t)(offset + 11U) <= rxSize)
    {
        if (wit_dmaBuffer[offset] != 0x55U)
        {
            offset++;
            continue;
        }

        checkSum = 0;
        for (uint8_t i = offset; i < (uint8_t)(offset + 10U); i++) {
            checkSum += wit_dmaBuffer[i];
        }

        if (checkSum == wit_dmaBuffer[offset + 10U])
        {
            wit_valid_frame_count++;
            if(wit_dmaBuffer[offset + 1U] == 0x51U)
            {
                wit_data.ax = (int16_t)((wit_dmaBuffer[offset+3U]<<8)|wit_dmaBuffer[offset+2U]) / 2.048; //mg
                wit_data.ay = (int16_t)((wit_dmaBuffer[offset+5U]<<8)|wit_dmaBuffer[offset+4U]) / 2.048; //mg
                wit_data.az = (int16_t)((wit_dmaBuffer[offset+7U]<<8)|wit_dmaBuffer[offset+6U]) / 2.048; //mg
                wit_data.temperature =  (int16_t)((wit_dmaBuffer[offset+9U]<<8)|wit_dmaBuffer[offset+8U]) / 100.0; //°C
            }
            else if(wit_dmaBuffer[offset + 1U] == 0x52U)
            {
                wit_data.gx = (int16_t)((wit_dmaBuffer[offset+3U]<<8)|wit_dmaBuffer[offset+2U]) / 16.384; //°/S
                wit_data.gy = (int16_t)((wit_dmaBuffer[offset+5U]<<8)|wit_dmaBuffer[offset+4U]) / 16.384; //°/S
                wit_data.gz = (int16_t)((wit_dmaBuffer[offset+7U]<<8)|wit_dmaBuffer[offset+6U]) / 16.384; //°/S
            }
            else if(wit_dmaBuffer[offset + 1U] == 0x53U)
            {
                wit_data.roll  = (int16_t)((wit_dmaBuffer[offset+3U]<<8)|wit_dmaBuffer[offset+2U]) / 32768.0 * 180.0; //°
                wit_data.pitch = (int16_t)((wit_dmaBuffer[offset+5U]<<8)|wit_dmaBuffer[offset+4U]) / 32768.0 * 180.0; //°
                wit_data.yaw   = (int16_t)((wit_dmaBuffer[offset+7U]<<8)|wit_dmaBuffer[offset+6U]) / 32768.0 * 180.0; //°
                wit_data.version = (int16_t)((wit_dmaBuffer[offset+9U]<<8)|wit_dmaBuffer[offset+8U]);
                wit_angle_frame_count++;
            }
            offset += 11U;
        }
        else
        {
            offset++;
        }
    }
    
    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_WIT_INST, dummy, 4);

    for (uint8_t i = 0; i < sizeof(wit_dmaBuffer); i++)
    {
        wit_dmaBuffer[i] = 0U;
    }
    DL_DMA_setDestAddr(DMA, DMA_WIT_CHAN_ID, (uint32_t) &wit_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_WIT_CHAN_ID, 32);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);
}
#endif

// void GROUP1_IRQHandler(void)
// {
//     switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
//         #if defined GPIO_MULTIPLE_GPIOA_INT_IIDX
//         case GPIO_MULTIPLE_GPIOA_INT_IIDX:
//             switch (DL_GPIO_getPendingInterrupt(GPIOA))
//             {
//                 #if (defined GPIO_MPU6050_PORT) && (GPIO_MPU6050_PORT == GPIOA)
//                 case GPIO_MPU6050_PIN_MPU6050_INT_IIDX:
//                     Read_Quad();
//                     break;
//                 #endif

//                 #if (defined GPIO_LSM6DSV16X_PORT) && (GPIO_LSM6DSV16X_PORT == GPIOA)
//                 case GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_IIDX:
//                     Read_LSM6DSV16X();
//                     break;
//                 #endif

//                 #if (defined GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT) && (GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT == GPIOA)
//                 case GPIO_VL53L0X_PIN_VL53L0X_GPIO1_IIDX:
//                     Read_VL53L0X();
//                     break;
//                 #endif

//                 default:
//                     break;
//             }
//         #endif

//         #if defined GPIO_MULTIPLE_GPIOB_INT_IIDX
//         case GPIO_MULTIPLE_GPIOB_INT_IIDX:
//             switch (DL_GPIO_getPendingInterrupt(GPIOB))
//             {
//                 #if (defined GPIO_MPU6050_PORT) && (GPIO_MPU6050_PORT == GPIOB)
//                 case GPIO_MPU6050_PIN_MPU6050_INT_IIDX:
//                     Read_Quad();
//                     break;
//                 #endif

//                 #if (defined GPIO_LSM6DSV16X_PORT) && (GPIO_LSM6DSV16X_PORT == GPIOB)
//                 case GPIO_LSM6DSV16X_PIN_LSM6DSV16X_INT_IIDX:
//                     Read_LSM6DSV16X();
//                     break;
//                 #endif

//                 #if (defined GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT) && (GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT == GPIOB)
//                 case GPIO_VL53L0X_PIN_VL53L0X_GPIO1_IIDX:
//                     Read_VL53L0X();
//                     break;
//                 #endif

//                 default:
//                     break;
//             }
//         #endif

//         #if defined GPIO_MPU6050_INT_IIDX
//             case GPIO_MPU6050_INT_IIDX:
//                 Read_Quad();
//                 break;
//         #endif

//         #if defined GPIO_LSM6DSV16X_INT_IIDX
//             case GPIO_LSM6DSV16X_INT_IIDX:
//                 Read_LSM6DSV16X();
//                 break;
//         #endif

//         #if defined GPIO_VL53L0X_INT_IIDX
//             case GPIO_VL53L0X_INT_IIDX:
//                 Read_VL53L0X();
//                 break;
//         #endif
//     }
// }
