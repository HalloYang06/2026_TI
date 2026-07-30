#ifndef HBALL_CAN_PORT_H
#define HBALL_CAN_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t tx_queued;
    uint32_t tx_confirmed;
    uint32_t tx_busy;
    uint32_t tx_failed;
    uint32_t rx_total;
    uint32_t rx_standard;
    uint32_t rx_extended;
    uint32_t rx_remote;
    uint32_t rx_fd_rejected;
    uint32_t rx_fifo_full;
    uint32_t rx_fifo_lost;
    uint32_t bus_off_events;
    uint32_t protocol_error_events;
    uint32_t message_ram_errors;
    uint32_t last_rx_id;
    uint32_t last_irq_status;
    uint16_t tx_error_count;
    uint16_t rx_error_count;
    uint8_t last_rx_extended;
    uint8_t last_rx_dlc;
    uint8_t last_rx_data[8];
    uint8_t initialized;
    uint8_t bus_off;
    uint8_t error_passive;
    uint8_t error_warning;
    uint32_t bus_off_recovery_attempts;
    uint32_t bus_off_recoveries;
} hball_can_port_stats_t;

extern volatile hball_can_port_stats_t g_hball_can_stats;

void hball_can_port_init(void);
void hball_can_port_tick_1ms(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
