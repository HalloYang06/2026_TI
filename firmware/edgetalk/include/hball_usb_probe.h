#ifndef HBALL_USB_PROBE_H
#define HBALL_USB_PROBE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HBALL_USB_PROBE_VERSION "0.1.0"
#define HBALL_USB_MAX_PAYLOAD 48U
#define HBALL_USB_LINE_CAPACITY 96U

typedef struct
{
    uint32_t sequence;
    char payload[HBALL_USB_MAX_PAYLOAD + 1U];
} hball_usb_ping_t;

bool hball_usb_parse_ping(
    const char *line, size_t length, hball_usb_ping_t *ping
);
size_t hball_usb_format_pong(
    char *destination,
    size_t capacity,
    uint32_t sequence,
    const char *payload
);
size_t hball_usb_format_ready(
    char *destination, size_t capacity, uint32_t sequence, uint32_t uptime_ms
);

#ifdef __cplusplus
}
#endif

#endif
