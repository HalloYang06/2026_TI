#ifndef _TRACK_H
#define _TRACK_H

#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "clock.h"
#include "motor.h"
#include "beeper.h"
#include "led.h"

extern volatile int start;

void track_start(uint8_t rounds);
void track_stop(void);
void my_track(void);

#endif
