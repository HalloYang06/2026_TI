#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "clock.h"
#include "../MPU6050/mpu6050.h"
#include "../BNO08X_UART_RVC/bno08x_uart_rvc.h"
#include "wit.h"
#include "../VL53L0X/vl53l0x.h"
#include "../LSM6DSV16X/lsm6dsv16x.h"
#include "hball_runtime_services.h"
#include "hball_runtime_target.h"
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
    hball_runtime_target_tick_isr();
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
static void wit_process_dma_chunk(void)
{
    const bool process_imu = hball_runtime_services_imu_enabled();
    uint16_t remaining;
    uint16_t received;
    uint8_t fifo_byte;

    DL_DMA_disableChannel(DMA, DMA_WIT_CHAN_ID);
    remaining = DL_DMA_getTransferSize(DMA, DMA_WIT_CHAN_ID);
    received = (remaining <= WIT_DMA_TRANSFER_SIZE)
        ? (uint16_t)(WIT_DMA_TRANSFER_SIZE - remaining)
        : 0U;
    if (process_imu)
    {
        (void)WIT_QueueBytesFromISR(wit_dmaBuffer, received);
    }

    while (DL_UART_isRXFIFOEmpty(UART_WIT_INST) == false)
    {
        fifo_byte = DL_UART_receiveData(UART_WIT_INST);
        if (process_imu)
        {
            (void)WIT_QueueBytesFromISR(&fifo_byte, 1U);
        }
    }

    /*
     * A 200 Hz three-report stream is 33 bytes per source period, so a
     * 32-byte DMA block can end one byte before a checksum. Preserve parser
     * state across blocks and re-arm on both DMA completion and UART idle.
     */
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0);
    DL_DMA_setDestAddr(
        DMA, DMA_WIT_CHAN_ID, (uint32_t)&wit_dmaBuffer[0]);
    DL_DMA_setTransferSize(
        DMA, DMA_WIT_CHAN_ID, WIT_DMA_TRANSFER_SIZE);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);
}

void UART_WIT_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART_WIT_INST) ==
        DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR)
    {
        wit_process_dma_chunk();
    }
}

void DMA_IRQHandler(void)
{
    if (DL_DMA_getPendingInterrupt(DMA) == DL_DMA_EVENT_IIDX_DMACH0)
    {
        wit_process_dma_chunk();
    }
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
