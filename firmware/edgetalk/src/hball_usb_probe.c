#include "hball_usb_probe.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static size_t hball_usb_trim_line_end(const char *line, size_t length)
{
    while ((length > 0U)
        && ((line[length - 1U] == '\n') || (line[length - 1U] == '\r')))
    {
        length--;
    }
    return length;
}

bool hball_usb_parse_ping(
    const char *line, size_t length, hball_usb_ping_t *ping
)
{
    size_t index;
    size_t payload_length;
    uint32_t sequence = 0U;

    if ((line == NULL) || (ping == NULL) || (length == 0U)
        || (length >= HBALL_USB_LINE_CAPACITY))
    {
        return false;
    }

    length = hball_usb_trim_line_end(line, length);
    if ((length < 6U) || (memcmp(line, "PING ", 5U) != 0))
    {
        return false;
    }

    index = 5U;
    if ((line[index] < '0') || (line[index] > '9'))
    {
        return false;
    }
    while ((index < length) && (line[index] >= '0') && (line[index] <= '9'))
    {
        uint32_t digit = (uint32_t)(line[index] - '0');

        if ((sequence > (UINT32_MAX / 10U))
            || ((sequence == (UINT32_MAX / 10U))
                && (digit > (UINT32_MAX % 10U))))
        {
            return false;
        }
        sequence = (sequence * 10U) + digit;
        index++;
    }

    if ((index < length) && (line[index] != ' '))
    {
        return false;
    }
    if (index < length)
    {
        index++;
    }

    payload_length = length - index;
    if (payload_length > HBALL_USB_MAX_PAYLOAD)
    {
        return false;
    }
    for (size_t payload_index = 0U;
         payload_index < payload_length;
         ++payload_index)
    {
        const unsigned char value = (unsigned char)line[index + payload_index];

        if ((value < 0x20U) || (value > 0x7EU))
        {
            return false;
        }
    }

    ping->sequence = sequence;
    if (payload_length > 0U)
    {
        memcpy(ping->payload, line + index, payload_length);
    }
    ping->payload[payload_length] = '\0';
    return true;
}

size_t hball_usb_format_pong(
    char *destination,
    size_t capacity,
    uint32_t sequence,
    const char *payload
)
{
    int written;

    if ((destination == NULL) || (capacity == 0U))
    {
        return 0U;
    }
    if (payload == NULL)
    {
        payload = "";
    }
    if (strlen(payload) > HBALL_USB_MAX_PAYLOAD)
    {
        destination[0] = '\0';
        return 0U;
    }

    written = (payload[0] == '\0')
        ? snprintf(destination, capacity, "PONG %" PRIu32 "\n", sequence)
        : snprintf(
            destination,
            capacity,
            "PONG %" PRIu32 " %s\n",
            sequence,
            payload
        );
    if ((written < 0) || ((size_t)written >= capacity))
    {
        destination[0] = '\0';
        return 0U;
    }
    return (size_t)written;
}

size_t hball_usb_format_ready(
    char *destination, size_t capacity, uint32_t sequence, uint32_t uptime_ms
)
{
    int written;

    if ((destination == NULL) || (capacity == 0U))
    {
        return 0U;
    }
    written = snprintf(
        destination,
        capacity,
        "HBALL_USB_READY %s %" PRIu32 " %" PRIu32 "\n",
        HBALL_USB_PROBE_VERSION,
        sequence,
        uptime_ms
    );
    if ((written < 0) || ((size_t)written >= capacity))
    {
        destination[0] = '\0';
        return 0U;
    }
    return (size_t)written;
}

bool hball_usb_parse_tune(
    const char *line, size_t length, hball_usb_tune_t *tune
)
{
    char buffer[HBALL_USB_LINE_CAPACITY];
    char extra;

    if ((line == NULL) || (tune == NULL) || (length == 0U)
        || (length >= sizeof(buffer)))
    {
        return false;
    }
    length = hball_usb_trim_line_end(line, length);
    memcpy(buffer, line, length);
    buffer[length] = '\0';
    return sscanf(
        buffer,
        "HBALL_TUNE %" SCNu32 " %31s %f %c",
        &tune->sequence,
        tune->name,
        &tune->value,
        &extra
    ) == 3;
}

size_t hball_usb_format_tune_ack(
    char *destination,
    size_t capacity,
    const hball_usb_tune_t *tune,
    bool accepted
)
{
    int written;

    if ((destination == NULL) || (capacity == 0U) || (tune == NULL))
    {
        return 0U;
    }
    written = snprintf(
        destination,
        capacity,
        "HBALL_TUNE_ACK %" PRIu32 " %s %.6g %s\n",
        tune->sequence,
        tune->name,
        (double)tune->value,
        accepted ? "OK" : "REJECT"
    );
    if ((written < 0) || ((size_t)written >= capacity))
    {
        destination[0] = '\0';
        return 0U;
    }
    return (size_t)written;
}
