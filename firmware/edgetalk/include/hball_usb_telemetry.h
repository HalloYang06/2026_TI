#ifndef HBALL_USB_TELEMETRY_H
#define HBALL_USB_TELEMETRY_H

#include "hball_log_protocol.h"

#include <stdbool.h>

bool hball_usb_telemetry_submit(const hball_log_record_t *record);

#endif
