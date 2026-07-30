#include "wit.h"

#include "wit_parser.h"

#include <stddef.h>

uint8_t wit_dmaBuffer[WIT_DMA_TRANSFER_SIZE];

volatile WIT_Data_t wit_data;
volatile uint32_t wit_rx_byte_count;
volatile uint32_t wit_valid_frame_count;
volatile uint32_t wit_checksum_error_count;
volatile uint32_t wit_unknown_frame_count;
volatile uint32_t wit_accel_frame_count;
volatile uint32_t wit_gyro_frame_count;
volatile uint32_t wit_angle_frame_count;

static wit_parser_t g_wit_parser;

static int16_t wit_load_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static void wit_decode_frame(const uint8_t frame[WIT_FRAME_SIZE])
{
    if (frame[1] == 0x51U)
    {
        wit_data.ax = (int16_t)(wit_load_i16_le(frame + 2U) / 2.048F);
        wit_data.ay = (int16_t)(wit_load_i16_le(frame + 4U) / 2.048F);
        wit_data.az = (int16_t)(wit_load_i16_le(frame + 6U) / 2.048F);
        wit_data.temperature = wit_load_i16_le(frame + 8U) / 100.0F;
        wit_accel_frame_count++;
    }
    else if (frame[1] == 0x52U)
    {
        wit_data.gx = (int16_t)(wit_load_i16_le(frame + 2U) / 16.384F);
        wit_data.gy = (int16_t)(wit_load_i16_le(frame + 4U) / 16.384F);
        wit_data.gz = (int16_t)(wit_load_i16_le(frame + 6U) / 16.384F);
        wit_gyro_frame_count++;
    }
    else if (frame[1] == 0x53U)
    {
        wit_data.roll = wit_load_i16_le(frame + 2U) / 32768.0F * 180.0F;
        wit_data.pitch = wit_load_i16_le(frame + 4U) / 32768.0F * 180.0F;
        wit_data.yaw = wit_load_i16_le(frame + 6U) / 32768.0F * 180.0F;
        wit_data.version = wit_load_i16_le(frame + 8U);
        wit_angle_frame_count++;
    }
    else
    {
        wit_unknown_frame_count++;
    }
}

void WIT_ProcessBytes(const uint8_t *data, uint16_t length)
{
    uint8_t frame[WIT_FRAME_SIZE];

    if (data == NULL)
    {
        return;
    }
    for (uint16_t i = 0U; i < length; ++i)
    {
        const wit_parser_event_t event =
            wit_parser_push(&g_wit_parser, data[i], frame);

        wit_rx_byte_count++;
        if (event == WIT_PARSER_VALID)
        {
            wit_valid_frame_count++;
            wit_decode_frame(frame);
        }
        else if (event == WIT_PARSER_CHECKSUM_ERROR)
        {
            wit_checksum_error_count++;
        }
    }
}

void WIT_Init(void)
{
    wit_rx_byte_count = 0U;
    wit_valid_frame_count = 0U;
    wit_checksum_error_count = 0U;
    wit_unknown_frame_count = 0U;
    wit_accel_frame_count = 0U;
    wit_gyro_frame_count = 0U;
    wit_angle_frame_count = 0U;
    wit_parser_init(&g_wit_parser);
    for (uint8_t i = 0U; i < sizeof(wit_dmaBuffer); ++i)
    {
        wit_dmaBuffer[i] = 0U;
    }

    DL_DMA_setSrcAddr(
        DMA, DMA_WIT_CHAN_ID, (uint32_t)(&UART_WIT_INST->RXDATA));
    DL_DMA_setDestAddr(
        DMA, DMA_WIT_CHAN_ID, (uint32_t)&wit_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_WIT_CHAN_ID, WIT_DMA_TRANSFER_SIZE);
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0);
    DL_DMA_enableInterrupt(DMA, DL_DMA_INTERRUPT_CHANNEL0);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);

    NVIC_ClearPendingIRQ(DMA_INT_IRQn);
    NVIC_SetPriority(DMA_INT_IRQn, 0U);
    NVIC_EnableIRQ(DMA_INT_IRQn);
    NVIC_ClearPendingIRQ(UART_WIT_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_WIT_INST_INT_IRQN);
}
