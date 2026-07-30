#ifndef HBALL_DIAGNOSTICS_CONFIG_H
#define HBALL_DIAGNOSTICS_CONFIG_H

/*
 * Polling UART output can block long enough to fill the CAN RX FIFO at the
 * integrated 720 frame/s telemetry rate.  Keep periodic diagnostics opt-in;
 * the FinSH status commands remain available for an operator-requested dump.
 */
#ifndef HBALL_PERIODIC_DIAGNOSTICS
#define HBALL_PERIODIC_DIAGNOSTICS 0
#endif

#endif
