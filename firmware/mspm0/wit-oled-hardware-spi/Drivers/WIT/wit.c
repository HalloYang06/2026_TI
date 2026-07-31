#include "wit.h"

#include "wit_jy901s_config.h"
#include "wit_byte_queue.h"
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
volatile uint32_t wit_queue_enqueued_byte_count;
volatile uint32_t wit_queue_dropped_byte_count;
volatile uint32_t wit_serviced_byte_count;
volatile uint16_t wit_queue_depth;
volatile uint16_t wit_queue_high_water;

static wit_parser_t g_wit_parser;
static wit_byte_queue_t g_wit_byte_queue;

static void wit_jy901s_write_register(uint8_t address, uint16_t value)
{
    uint8_t command[WIT_JY901S_COMMAND_SIZE];

    wit_jy901s_build_write_command(address, value, command);
    for (uint8_t i = 0U; i < WIT_JY901S_COMMAND_SIZE; ++i)
    {
        DL_UART_Main_transmitDataBlocking(UART_WIT_INST, command[i]);
    }
    while (DL_UART_Main_isBusy(UART_WIT_INST))
    {
    }
}

static void wit_jy901s_enable_control_reports(void)
{
    /*
     * WITMOTION's official JY901 protocol defines KEY=0x69,
     * KEY_UNLOCK=0xB588 and RSW=0x02. WitSetContent unlocks, waits 1 ms,
     * then writes RSW. 0x000E selects acceleration, gyro and angle only.
     * Do this on every MSPM0 boot instead of saving repeatedly to the sensor.
     * Source:
     * https://github.com/WITMOTION/WitStandardProtocol_JY901/blob/main/Arduino/Arduino_sdk/wit_c_sdk.c#L486-L498
     */
    wit_jy901s_write_register(WIT_JY901S_REG_KEY, WIT_JY901S_KEY_UNLOCK);
    delay_cycles(CPUCLK_FREQ / 1000U);
    wit_jy901s_write_register(
        WIT_JY901S_REG_OUTPUT_CONTENT,
        WIT_JY901S_OUTPUT_ACCEL_GYRO_ANGLE);
    delay_cycles(CPUCLK_FREQ / 1000U);

    while (DL_UART_isRXFIFOEmpty(UART_WIT_INST) == false)
    {
        (void)DL_UART_receiveData(UART_WIT_INST);
    }
}

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

uint16_t WIT_QueueBytesFromISR(const uint8_t *data, uint16_t length)
{
    const uint16_t accepted = wit_byte_queue_push(
        &g_wit_byte_queue, data, length
    );

    wit_queue_enqueued_byte_count += accepted;
    wit_queue_dropped_byte_count =
        wit_byte_queue_dropped(&g_wit_byte_queue);
    wit_queue_depth = wit_byte_queue_depth(&g_wit_byte_queue);
    wit_queue_high_water = wit_byte_queue_high_water(&g_wit_byte_queue);
    return accepted;
}

uint16_t WIT_Service(uint16_t max_bytes)
{
    uint8_t buffer[WIT_FOREGROUND_BUDGET_PER_SERVICE];
    uint16_t serviced = 0U;

    while (serviced < max_bytes)
    {
        const uint16_t remaining = (uint16_t)(max_bytes - serviced);
        const uint16_t capacity =
            (remaining < sizeof(buffer)) ? remaining : sizeof(buffer);
        const uint16_t count = wit_byte_queue_pop(
            &g_wit_byte_queue, buffer, capacity
        );

        if (count == 0U)
        {
            break;
        }
        WIT_ProcessBytes(buffer, count);
        serviced = (uint16_t)(serviced + count);
    }
    wit_serviced_byte_count += serviced;
    wit_queue_depth = wit_byte_queue_depth(&g_wit_byte_queue);
    return serviced;
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
    wit_queue_enqueued_byte_count = 0U;
    wit_queue_dropped_byte_count = 0U;
    wit_serviced_byte_count = 0U;
    wit_queue_depth = 0U;
    wit_queue_high_water = 0U;
    wit_parser_init(&g_wit_parser);
    wit_byte_queue_init(&g_wit_byte_queue);
    for (uint8_t i = 0U; i < sizeof(wit_dmaBuffer); ++i)
    {
        wit_dmaBuffer[i] = 0U;
    }

    wit_jy901s_enable_control_reports();

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
