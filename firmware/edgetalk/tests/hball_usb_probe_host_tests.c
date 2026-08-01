#include "hball_usb_probe.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

static void test_ping_parse_and_pong_format(void)
{
    hball_usb_ping_t ping;
    char response[HBALL_USB_LINE_CAPACITY];
    const char request[] = "PING 42 camera frame 7\r\n";
    size_t response_length;

    assert(hball_usb_parse_ping(request, sizeof(request) - 1U, &ping));
    assert(ping.sequence == 42U);
    assert(strcmp(ping.payload, "camera frame 7") == 0);

    response_length = hball_usb_format_pong(
        response, sizeof(response), ping.sequence, ping.payload
    );
    assert(response_length == strlen("PONG 42 camera frame 7\n"));
    assert(strcmp(response, "PONG 42 camera frame 7\n") == 0);
}

static void test_ping_without_payload_is_valid(void)
{
    hball_usb_ping_t ping;
    char response[HBALL_USB_LINE_CAPACITY];

    assert(hball_usb_parse_ping("PING 0\n", 7U, &ping));
    assert(ping.sequence == 0U);
    assert(ping.payload[0] == '\0');
    assert(hball_usb_format_pong(
        response, sizeof(response), ping.sequence, ping.payload
    ) == strlen("PONG 0\n"));
    assert(strcmp(response, "PONG 0\n") == 0);
}

static void test_invalid_or_unsafe_ping_is_rejected(void)
{
    hball_usb_ping_t ping;
    char oversized[HBALL_USB_LINE_CAPACITY + 8U];

    memset(oversized, 'A', sizeof(oversized));
    assert(!hball_usb_parse_ping("PONG 1 x\n", 9U, &ping));
    assert(!hball_usb_parse_ping("PING 4294967296\n", 16U, &ping));
    assert(!hball_usb_parse_ping("PING 1 bad\x01" "data\n", 16U, &ping));
    assert(!hball_usb_parse_ping(oversized, sizeof(oversized), &ping));
}

static void test_ready_format_contains_version_sequence_and_uptime(void)
{
    char ready[HBALL_USB_LINE_CAPACITY];
    char too_small[8];

    assert(hball_usb_format_ready(
        ready, sizeof(ready), 9U, 12345U
    ) == strlen("HBALL_USB_READY 0.1.0 9 12345\n"));
    assert(strcmp(ready, "HBALL_USB_READY 0.1.0 9 12345\n") == 0);
    assert(hball_usb_format_ready(
        too_small, sizeof(too_small), 9U, 12345U
    ) == 0U);
}

static void test_runtime_tune_parse_and_ack(void)
{
    hball_usb_tune_t tune;
    char response[HBALL_USB_LINE_CAPACITY];
    const char valid[] = "HBALL_TUNE 17 settle_capture_deg 2.5\n";
    const char invalid[] = "HBALL_TUNE 17 run_deg 2 extra\n";

    assert(hball_usb_parse_tune(
        valid, sizeof(valid) - 1U, &tune
    ));
    assert(tune.sequence == 17U);
    assert(strcmp(tune.name, "settle_capture_deg") == 0);
    assert(tune.value == 2.5F);
    assert(hball_usb_format_tune_ack(
        response, sizeof(response), &tune, true
    ) > 0U);
    assert(strcmp(
        response,
        "HBALL_TUNE_ACK 17 settle_capture_deg 2.5 OK\n"
    ) == 0);
    assert(!hball_usb_parse_tune(
        invalid, sizeof(invalid) - 1U, &tune
    ));
}

int main(void)
{
    test_ping_parse_and_pong_format();
    test_ping_without_payload_is_valid();
    test_invalid_or_unsafe_ping_is_rejected();
    test_ready_format_contains_version_sequence_and_uptime();
    test_runtime_tune_parse_and_ack();
    return 0;
}
