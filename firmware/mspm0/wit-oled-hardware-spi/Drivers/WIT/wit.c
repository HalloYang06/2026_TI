#include "wit.h"

uint8_t wit_dmaBuffer[33];

volatile WIT_Data_t wit_data;
volatile uint32_t wit_rx_byte_count;
volatile uint32_t wit_valid_frame_count;
volatile uint32_t wit_accel_frame_count;
volatile uint32_t wit_gyro_frame_count;
volatile uint32_t wit_angle_frame_count;

void WIT_Init(void)
{
    wit_rx_byte_count = 0;
    wit_valid_frame_count = 0;
    wit_accel_frame_count = 0;
    wit_gyro_frame_count = 0;
    wit_angle_frame_count = 0;
    for (uint8_t i = 0; i < sizeof(wit_dmaBuffer); i++)
    {
        wit_dmaBuffer[i] = 0U;
    }
    DL_DMA_setSrcAddr(DMA, DMA_WIT_CHAN_ID, (uint32_t)(&UART_WIT_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_WIT_CHAN_ID, (uint32_t) &wit_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_WIT_CHAN_ID, 32);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);

    NVIC_ClearPendingIRQ(UART_WIT_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_WIT_INST_INT_IRQN);
}
