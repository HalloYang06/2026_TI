#include "hball_usb_probe.h"

#include <finsh.h>
#include <rtthread.h>
#include <string.h>

#include "USB.h"
#include "USB_CDC.h"
#include "hball_rate_meter.h"
#include "hball_vision_protocol.h"
#if HBALL_INTEGRATED_SHADOW
#include "hball_m33_inputs.h"
#endif

#define HBALL_USB_THREAD_STACK_SIZE 4096U
#define HBALL_USB_THREAD_PRIORITY 20U
#define HBALL_USB_THREAD_TIMESLICE 10U
#define HBALL_USB_POLL_MS 2U
#define HBALL_USB_DISCONNECTED_POLL_MS 20U
#define HBALL_USB_READY_PERIOD_MS 1000U

#define HBALL_USB_VID 0x058BU
#define HBALL_USB_PID 0x0282U
#define HBALL_USB_ENABLE_FLAG 0U
#define HBALL_USB_BULK_INTERVAL 0U
#define HBALL_USB_INT_INTERVAL 64U

typedef struct
{
    volatile unsigned state;
    volatile rt_bool_t configured;
    rt_bool_t attached_previous;
    rt_bool_t configured_previous;
    rt_uint32_t connected_total;
    rt_uint32_t configured_total;
    rt_uint32_t disconnected_total;
    rt_uint32_t open_total;
    rt_uint32_t ready_tx_total;
    rt_uint32_t pong_tx_total;
    rt_uint32_t tx_failure_total;
    rt_uint32_t rx_failure_total;
    rt_uint32_t ping_rx_total;
    rt_uint32_t invalid_rx_total;
    rt_uint32_t overflow_total;
    rt_uint32_t vision_rx_total;
    rt_uint32_t vision_position_valid_total;
    rt_uint32_t vision_duplicate_total;
    rt_uint32_t vision_out_of_order_total;
    rt_uint32_t vision_gap_total;
    rt_uint32_t last_vision_rx_ms;
    rt_uint32_t last_vision_sequence;
    rt_uint16_t last_vision_flags;
    float last_vision_position_m;
    float last_vision_confidence;
    rt_bool_t vision_sequence_initialized;
    rt_bool_t binary_mode;
} hball_usb_stats_t;

static const USB_DEVICE_INFO g_hball_usb_device_info = {
    HBALL_USB_VID,
    HBALL_USB_PID,
    "Infineon Technologies",
    "HBall EdgeTalk CDC Probe",
    "HBALL-PROBE"
};

static USB_CDC_HANDLE g_hball_usb_cdc_handle = -1;
static hball_usb_stats_t g_hball_usb_stats;
static hball_vision_stream_t g_hball_vision_stream;
static hball_rate_meter_t g_hball_vision_rate;
static rt_thread_t g_hball_usb_thread = RT_NULL;

/*
 * Endpoint layout and startup order follow Infineon's PSoC Edge CDC echo
 * example at commit 42fbdaeeac61c8b9eae049855b488f0862d9385c.
 * Source: https://github.com/Infineon/mtb-example-psoc-edge-usb-device-cdc-echo/blob/42fbdaeeac61c8b9eae049855b488f0862d9385c/proj_cm33_ns/main.c
 */
static void hball_usb_add_cdc(void)
{
    static U8 out_buffer[USB_HS_BULK_MAX_PACKET_SIZE];
    USB_CDC_INIT_DATA init_data;
    USB_ADD_EP_INFO ep_bulk_in;
    USB_ADD_EP_INFO ep_bulk_out;
    USB_ADD_EP_INFO ep_int_in;

    memset(&init_data, 0, sizeof(init_data));
    memset(&ep_bulk_in, 0, sizeof(ep_bulk_in));
    memset(&ep_bulk_out, 0, sizeof(ep_bulk_out));
    memset(&ep_int_in, 0, sizeof(ep_int_in));

    ep_bulk_in.Flags = HBALL_USB_ENABLE_FLAG;
    ep_bulk_in.InDir = USB_DIR_IN;
    ep_bulk_in.Interval = HBALL_USB_BULK_INTERVAL;
    ep_bulk_in.MaxPacketSize = USB_HS_BULK_MAX_PACKET_SIZE;
    ep_bulk_in.TransferType = USB_TRANSFER_TYPE_BULK;
    init_data.EPIn = USBD_AddEPEx(&ep_bulk_in, RT_NULL, 0U);

    ep_bulk_out.Flags = HBALL_USB_ENABLE_FLAG;
    ep_bulk_out.InDir = USB_DIR_OUT;
    ep_bulk_out.Interval = HBALL_USB_BULK_INTERVAL;
    ep_bulk_out.MaxPacketSize = USB_HS_BULK_MAX_PACKET_SIZE;
    ep_bulk_out.TransferType = USB_TRANSFER_TYPE_BULK;
    init_data.EPOut = USBD_AddEPEx(
        &ep_bulk_out, out_buffer, sizeof(out_buffer)
    );

    ep_int_in.Flags = HBALL_USB_ENABLE_FLAG;
    ep_int_in.InDir = USB_DIR_IN;
    ep_int_in.Interval = HBALL_USB_INT_INTERVAL;
    ep_int_in.MaxPacketSize = USB_HS_INT_MAX_PACKET_SIZE;
    ep_int_in.TransferType = USB_TRANSFER_TYPE_INT;
    init_data.EPInt = USBD_AddEPEx(&ep_int_in, RT_NULL, 0U);

    g_hball_usb_cdc_handle = USBD_CDC_Add(&init_data);
}

static int hball_usb_device_init(void)
{
    USBD_Init();
    hball_usb_add_cdc();
    USBD_SetDeviceInfo(&g_hball_usb_device_info);
    USBD_Start();
    return (g_hball_usb_cdc_handle < 0) ? -RT_ERROR : RT_EOK;
}

static rt_bool_t hball_usb_poll_state(void)
{
    const unsigned state = USBD_GetState();
    const rt_bool_t attached = ((state & USB_STAT_ATTACHED) != 0U)
        ? RT_TRUE
        : RT_FALSE;
    const rt_bool_t configured =
        (USB_STAT_CONFIGURED
            == (state & (USB_STAT_CONFIGURED | USB_STAT_SUSPENDED)))
        ? RT_TRUE
        : RT_FALSE;

    if (attached && !g_hball_usb_stats.attached_previous)
    {
        g_hball_usb_stats.connected_total++;
    }
    if (!attached && g_hball_usb_stats.attached_previous)
    {
        g_hball_usb_stats.disconnected_total++;
    }
    if (configured && !g_hball_usb_stats.configured_previous)
    {
        g_hball_usb_stats.configured_total++;
        g_hball_usb_stats.open_total++;
    }

    g_hball_usb_stats.state = state;
    g_hball_usb_stats.configured = configured;
    g_hball_usb_stats.attached_previous = attached;
    g_hball_usb_stats.configured_previous = configured;
    return configured;
}

static rt_bool_t hball_usb_write(const char *message, rt_size_t length)
{
    int written;

    if ((message == RT_NULL) || (length == 0U)
        || !g_hball_usb_stats.configured
        || (g_hball_usb_cdc_handle < 0))
    {
        g_hball_usb_stats.tx_failure_total++;
        return RT_FALSE;
    }

    written = USBD_CDC_Write(
        g_hball_usb_cdc_handle, message, (unsigned)length, 0
    );
    if (written != (int)length)
    {
        g_hball_usb_stats.tx_failure_total++;
        return RT_FALSE;
    }
    (void)USBD_CDC_WaitForTX(g_hball_usb_cdc_handle, 0U);
    return RT_TRUE;
}

static void hball_usb_process_line(const char *line, rt_size_t length)
{
    hball_usb_ping_t ping;
    char response[HBALL_USB_LINE_CAPACITY];
    size_t response_length;

    if (!hball_usb_parse_ping(line, length, &ping))
    {
        g_hball_usb_stats.invalid_rx_total++;
        return;
    }
    g_hball_usb_stats.ping_rx_total++;
    response_length = hball_usb_format_pong(
        response, sizeof(response), ping.sequence, ping.payload
    );
    if ((response_length > 0U)
        && hball_usb_write(response, (rt_size_t)response_length))
    {
        g_hball_usb_stats.pong_tx_total++;
    }
}

static int hball_usb_receive(rt_uint8_t *chunk, rt_size_t capacity)
{
    return USBD_CDC_Receive(
        g_hball_usb_cdc_handle,
        chunk,
        (unsigned)capacity, 0
    );
}

static void hball_usb_accept_vision(
    const hball_vision_measurement_t *measurement, void *context
)
{
    rt_uint32_t sequence_delta;
    rt_uint32_t receive_ms;

    RT_UNUSED(context);
    g_hball_usb_stats.binary_mode = RT_TRUE;
    if (g_hball_usb_stats.vision_sequence_initialized)
    {
        sequence_delta = measurement->sequence
            - g_hball_usb_stats.last_vision_sequence;
        if (sequence_delta == 0U)
        {
            g_hball_usb_stats.vision_duplicate_total++;
            return;
        }
        if (sequence_delta >= UINT32_C(0x80000000))
        {
            g_hball_usb_stats.vision_out_of_order_total++;
            return;
        }
        if (sequence_delta > 1U)
        {
            g_hball_usb_stats.vision_gap_total += sequence_delta - 1U;
        }
    }

    g_hball_usb_stats.vision_sequence_initialized = RT_TRUE;
    g_hball_usb_stats.vision_rx_total++;
    receive_ms = (rt_uint32_t)rt_tick_get_millisecond();
    hball_rate_meter_accept(&g_hball_vision_rate, receive_ms);
    if ((measurement->flags & HBALL_VISION_FLAG_POSITION_VALID) != 0U)
    {
        g_hball_usb_stats.vision_position_valid_total++;
    }
    g_hball_usb_stats.last_vision_sequence = measurement->sequence;
    g_hball_usb_stats.last_vision_flags = measurement->flags;
    g_hball_usb_stats.last_vision_position_m = measurement->ball_position_m;
    g_hball_usb_stats.last_vision_confidence = measurement->confidence;
    g_hball_usb_stats.last_vision_rx_ms = receive_ms;
#if HBALL_INTEGRATED_SHADOW
    (void)hball_m33_inputs_publish_vision(
        measurement, g_hball_usb_stats.last_vision_rx_ms
    );
#endif
}

static void hball_usb_session(void)
{
    char line[HBALL_USB_LINE_CAPACITY];
    rt_uint8_t chunk[USB_HS_BULK_MAX_PACKET_SIZE];
    rt_size_t line_length = 0U;
    rt_bool_t discard_until_newline = RT_FALSE;
    rt_uint32_t ready_sequence = 0U;
    rt_uint32_t last_ready_ms =
        (rt_uint32_t)rt_tick_get_millisecond() - HBALL_USB_READY_PERIOD_MS;

    g_hball_vision_stream.length = 0U;
    hball_rate_meter_init(&g_hball_vision_rate);
    g_hball_usb_stats.binary_mode = RT_FALSE;
    g_hball_usb_stats.vision_sequence_initialized = RT_FALSE;

    while (hball_usb_poll_state())
    {
        const rt_uint32_t now_ms =
            (rt_uint32_t)rt_tick_get_millisecond();
        int received;

        if (!g_hball_usb_stats.binary_mode
            && ((rt_uint32_t)(now_ms - last_ready_ms)
                >= HBALL_USB_READY_PERIOD_MS))
        {
            char ready[HBALL_USB_LINE_CAPACITY];
            const size_t ready_length = hball_usb_format_ready(
                ready, sizeof(ready), ready_sequence, now_ms
            );

            last_ready_ms = now_ms;
            ready_sequence++;
            if ((ready_length > 0U)
                && hball_usb_write(ready, (rt_size_t)ready_length))
            {
                g_hball_usb_stats.ready_tx_total++;
            }
        }

        received = hball_usb_receive(chunk, sizeof(chunk));
        if (received < 0)
        {
            g_hball_usb_stats.rx_failure_total++;
            rt_thread_mdelay(HBALL_USB_POLL_MS);
            continue;
        }
        {
            const rt_uint32_t decoded_before =
                g_hball_vision_stream.accepted_total
                + g_hball_vision_stream.crc_failure_total
                + g_hball_vision_stream.header_failure_total
                + g_hball_vision_stream.range_failure_total;

            (void)hball_vision_stream_push(
                &g_hball_vision_stream,
                chunk,
                (rt_size_t)received,
                hball_usb_accept_vision,
                RT_NULL
            );
            if ((g_hball_vision_stream.length > 0U)
                || (decoded_before
                    != (g_hball_vision_stream.accepted_total
                        + g_hball_vision_stream.crc_failure_total
                        + g_hball_vision_stream.header_failure_total
                        + g_hball_vision_stream.range_failure_total)))
            {
                g_hball_usb_stats.binary_mode = RT_TRUE;
            }
        }
        if (g_hball_usb_stats.binary_mode
            || (g_hball_vision_stream.length > 0U))
        {
            continue;
        }
        for (int index = 0; index < received; ++index)
        {
            const char value = (char)chunk[index];

            if (discard_until_newline)
            {
                if (value == '\n')
                {
                    discard_until_newline = RT_FALSE;
                }
                continue;
            }
            if (line_length >= (sizeof(line) - 1U))
            {
                line_length = 0U;
                discard_until_newline = RT_TRUE;
                g_hball_usb_stats.overflow_total++;
                continue;
            }
            line[line_length++] = value;
            if (value == '\n')
            {
                line[line_length] = '\0';
                hball_usb_process_line(line, line_length);
                line_length = 0U;
            }
        }
    }
}

static void hball_usb_thread_entry(void *parameter)
{
    int init_result;

    RT_UNUSED(parameter);
    init_result = hball_usb_device_init();
    rt_kprintf(
        "[hball-usb] init version=%s stack=emusb result=%d handle=%d\n",
        HBALL_USB_PROBE_VERSION,
        init_result,
        g_hball_usb_cdc_handle
    );
    if (init_result != RT_EOK)
    {
        return;
    }

    while (1)
    {
        if (hball_usb_poll_state())
        {
            hball_usb_session();
        }
        else
        {
            rt_thread_mdelay(HBALL_USB_DISCONNECTED_POLL_MS);
        }
    }
}

static void hball_usb_status(void)
{
    const rt_uint32_t now_ms = (rt_uint32_t)rt_tick_get_millisecond();
    const rt_uint32_t vision_age_ms = g_hball_usb_stats.vision_rx_total > 0U
        ? now_ms - g_hball_usb_stats.last_vision_rx_ms
        : UINT32_MAX;
    const long position_um =
        (long)(g_hball_usb_stats.last_vision_position_m * 1000000.0F);
    const long confidence_permille =
        (long)(g_hball_usb_stats.last_vision_confidence * 1000.0F);
    const rt_uint32_t vision_rate_x10 = hball_rate_meter_hz_x10(
        &g_hball_vision_rate, now_ms
    );
    const rt_uint32_t vision_bytes_s =
        (vision_rate_x10 * HBALL_VISION_FRAME_SIZE) / 10U;

    (void)hball_usb_poll_state();
    rt_kprintf(
        "[hball-usb] version=%s state=0x%02lx configured=%d conn=%lu cfg=%lu disc=%lu open=%lu\n",
        HBALL_USB_PROBE_VERSION,
        (unsigned long)g_hball_usb_stats.state,
        (int)g_hball_usb_stats.configured,
        (unsigned long)g_hball_usb_stats.connected_total,
        (unsigned long)g_hball_usb_stats.configured_total,
        (unsigned long)g_hball_usb_stats.disconnected_total,
        (unsigned long)g_hball_usb_stats.open_total
    );
    rt_kprintf(
        "[hball-usb] ready_tx=%lu ping_rx=%lu pong_tx=%lu tx_fail=%lu rx_fail=%lu invalid=%lu overflow=%lu actuator_tx=0\n",
        (unsigned long)g_hball_usb_stats.ready_tx_total,
        (unsigned long)g_hball_usb_stats.ping_rx_total,
        (unsigned long)g_hball_usb_stats.pong_tx_total,
        (unsigned long)g_hball_usb_stats.tx_failure_total,
        (unsigned long)g_hball_usb_stats.rx_failure_total,
        (unsigned long)g_hball_usb_stats.invalid_rx_total,
        (unsigned long)g_hball_usb_stats.overflow_total
    );
    rt_kprintf(
        "[hball-usb] vision_rx=%lu position_valid=%lu dup=%lu ooo=%lu gap=%lu vision_crc=%lu header=%lu range=%lu discard=%lu\n",
        (unsigned long)g_hball_usb_stats.vision_rx_total,
        (unsigned long)g_hball_usb_stats.vision_position_valid_total,
        (unsigned long)g_hball_usb_stats.vision_duplicate_total,
        (unsigned long)g_hball_usb_stats.vision_out_of_order_total,
        (unsigned long)g_hball_usb_stats.vision_gap_total,
        (unsigned long)g_hball_vision_stream.crc_failure_total,
        (unsigned long)g_hball_vision_stream.header_failure_total,
        (unsigned long)g_hball_vision_stream.range_failure_total,
        (unsigned long)g_hball_vision_stream.discarded_byte_total
    );
    rt_kprintf(
        "[hball-usb] binary=%d seq=%lu flags=0x%04x age_ms=%lu pos_um=%ld confidence_permille=%ld\n",
        (int)g_hball_usb_stats.binary_mode,
        (unsigned long)g_hball_usb_stats.last_vision_sequence,
        (unsigned)g_hball_usb_stats.last_vision_flags,
        (unsigned long)vision_age_ms,
        position_um,
        confidence_permille
    );
    rt_kprintf(
        "[hball-usb] vision_rate_x10=%lu vision_bytes_s=%lu frame_bytes=%u target_hz=120 accept_hz=240\n",
        (unsigned long)vision_rate_x10,
        (unsigned long)vision_bytes_s,
        (unsigned int)HBALL_VISION_FRAME_SIZE
    );
}
MSH_CMD_EXPORT(hball_usb_status, show read-only H-ball USB CDC diagnostics);

static int hball_usb_cdc_start(void)
{
    g_hball_usb_thread = rt_thread_create(
        "hball_usb",
        hball_usb_thread_entry,
        RT_NULL,
        HBALL_USB_THREAD_STACK_SIZE,
        HBALL_USB_THREAD_PRIORITY,
        HBALL_USB_THREAD_TIMESLICE
    );
    if (g_hball_usb_thread == RT_NULL)
    {
        return -RT_ERROR;
    }
    rt_thread_startup(g_hball_usb_thread);
    return RT_EOK;
}
INIT_APP_EXPORT(hball_usb_cdc_start);
