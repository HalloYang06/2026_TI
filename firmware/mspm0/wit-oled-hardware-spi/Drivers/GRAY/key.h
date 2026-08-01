#ifndef _KEY_H
#define _KEY_H

#include "ti_msp_dl_config.h"
#include "main.h"

#define TASK_KEY_EXECUTE_PIN GPIO_KEY_KEY_1_PIN /* SW1, PA08 */
#define TASK_KEY_SELECT_PIN GPIO_KEY_KEY_3_PIN  /* SW3, PA09 */

typedef enum
{
    TASK_KEY_EVENT_NONE = 0,
    TASK_KEY_EVENT_SELECT,
    TASK_KEY_EVENT_EXECUTE
} task_key_event_t;

int get_keynum(void);
task_key_event_t get_task_key_event(void);
uint8_t get_q3_level_key_event(void);

#endif
