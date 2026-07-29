#ifndef _MAIN_H_
#define _MAIN_H_

extern volatile int start;
extern int round_number;

#include "clock.h"
#include "interrupt.h"

#include "oled_hardware_spi.h"
#include "wit.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "gray.h"
#include "uart_vofa.h"
#include "key.h"
#include "track.h"
#include "beeper.h"
#include "led.h"

#endif  /* #ifndef _MAIN_H_ */
