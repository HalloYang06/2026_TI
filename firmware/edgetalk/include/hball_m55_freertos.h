#ifndef HBALL_M55_FREERTOS_H
#define HBALL_M55_FREERTOS_H

#include "FreeRTOS.h"
#include "hball_m55_ui.h"

#ifdef __cplusplus
extern "C" {
#endif

BaseType_t hball_m55_freertos_start(void);

/* Override in the board LVGL port; called by the control task at 10 Hz. */
void hball_m55_platform_ui_10hz(const hball_m55_ui_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
