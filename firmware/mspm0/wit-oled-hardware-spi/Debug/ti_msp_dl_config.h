/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     80000000



/* Defines for PWM_A */
#define PWM_A_INST                                                         TIMA0
#define PWM_A_INST_IRQHandler                                   TIMA0_IRQHandler
#define PWM_A_INST_INT_IRQN                                     (TIMA0_INT_IRQn)
#define PWM_A_INST_CLK_FREQ                                             10000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_A_C0_PORT                                                 GPIOA
#define GPIO_PWM_A_C0_PIN                                          DL_GPIO_PIN_0
#define GPIO_PWM_A_C0_IOMUX                                       (IOMUX_PINCM1)
#define GPIO_PWM_A_C0_IOMUX_FUNC                      IOMUX_PINCM1_PF_TIMA0_CCP0
#define GPIO_PWM_A_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_A_C1_PORT                                                 GPIOA
#define GPIO_PWM_A_C1_PIN                                          DL_GPIO_PIN_1
#define GPIO_PWM_A_C1_IOMUX                                       (IOMUX_PINCM2)
#define GPIO_PWM_A_C1_IOMUX_FUNC                      IOMUX_PINCM2_PF_TIMA0_CCP1
#define GPIO_PWM_A_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMA1)
#define TIMER_0_INST_IRQHandler                                 TIMA1_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMA1_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                           (999U)
/* Defines for TIMER_1 */
#define TIMER_1_INST                                                     (TIMG0)
#define TIMER_1_INST_IRQHandler                                 TIMG0_IRQHandler
#define TIMER_1_INST_INT_IRQN                                   (TIMG0_INT_IRQn)
#define TIMER_1_INST_LOAD_VALUE                                         (49999U)



/* Defines for UART_1 */
#define UART_1_INST                                                        UART0
#define UART_1_INST_FREQUENCY                                           40000000
#define UART_1_INST_IRQHandler                                  UART0_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOA
#define GPIO_UART_1_TX_PORT                                                GPIOA
#define GPIO_UART_1_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_1_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_1_BAUD_RATE                                                  (9600)
#define UART_1_IBRD_40_MHZ_9600_BAUD                                       (260)
#define UART_1_FBRD_40_MHZ_9600_BAUD                                        (27)
/* Defines for UART_0 */
#define UART_0_INST                                                        UART1
#define UART_0_INST_FREQUENCY                                           40000000
#define UART_0_INST_IRQHandler                                  UART1_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOB
#define GPIO_UART_0_TX_PORT                                                GPIOB
#define GPIO_UART_0_RX_PIN                                         DL_GPIO_PIN_5
#define GPIO_UART_0_TX_PIN                                         DL_GPIO_PIN_4
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM18)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM17)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM18_PF_UART1_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM17_PF_UART1_TX
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_40_MHZ_9600_BAUD                                       (260)
#define UART_0_FBRD_40_MHZ_9600_BAUD                                        (27)
/* Defines for UART_WIT */
#define UART_WIT_INST                                                      UART3
#define UART_WIT_INST_FREQUENCY                                         40000000
#define UART_WIT_INST_IRQHandler                                UART3_IRQHandler
#define UART_WIT_INST_INT_IRQN                                    UART3_INT_IRQn
#define GPIO_UART_WIT_RX_PORT                                              GPIOB
#define GPIO_UART_WIT_TX_PORT                                              GPIOB
#define GPIO_UART_WIT_RX_PIN                                       DL_GPIO_PIN_3
#define GPIO_UART_WIT_TX_PIN                                       DL_GPIO_PIN_2
#define GPIO_UART_WIT_IOMUX_RX                                   (IOMUX_PINCM16)
#define GPIO_UART_WIT_IOMUX_TX                                   (IOMUX_PINCM15)
#define GPIO_UART_WIT_IOMUX_RX_FUNC                    IOMUX_PINCM16_PF_UART3_RX
#define GPIO_UART_WIT_IOMUX_TX_FUNC                    IOMUX_PINCM15_PF_UART3_TX
#define UART_WIT_BAUD_RATE                                              (115200)
#define UART_WIT_IBRD_40_MHZ_115200_BAUD                                    (21)
#define UART_WIT_FBRD_40_MHZ_115200_BAUD                                    (45)




/* Defines for SPI_LCD */
#define SPI_LCD_INST                                                       SPI1
#define SPI_LCD_INST_IRQHandler                                 SPI1_IRQHandler
#define SPI_LCD_INST_INT_IRQN                                     SPI1_INT_IRQn
#define GPIO_SPI_LCD_PICO_PORT                                            GPIOB
#define GPIO_SPI_LCD_PICO_PIN                                     DL_GPIO_PIN_8
#define GPIO_SPI_LCD_IOMUX_PICO                                 (IOMUX_PINCM25)
#define GPIO_SPI_LCD_IOMUX_PICO_FUNC                 IOMUX_PINCM25_PF_SPI1_PICO
/* GPIO configuration for SPI_LCD */
#define GPIO_SPI_LCD_SCLK_PORT                                            GPIOB
#define GPIO_SPI_LCD_SCLK_PIN                                     DL_GPIO_PIN_9
#define GPIO_SPI_LCD_IOMUX_SCLK                                 (IOMUX_PINCM26)
#define GPIO_SPI_LCD_IOMUX_SCLK_FUNC                 IOMUX_PINCM26_PF_SPI1_SCLK
#define GPIO_SPI_LCD_CD_PORT                                              GPIOA
#define GPIO_SPI_LCD_CD_PIN                                      DL_GPIO_PIN_25
#define GPIO_SPI_LCD_IOMUX_CD                                   (IOMUX_PINCM55)
#define GPIO_SPI_LCD_IOMUX_CD_FUNC           IOMUX_PINCM55_PF_SPI1_CS3_CD_POCI3



/* Defines for DMA_WIT */
#define DMA_WIT_CHAN_ID                                                      (0)
#define UART_WIT_INST_DMA_TRIGGER                            (DMA_UART3_RX_TRIG)


/* Port definition for Pin Group GPIO_BEEPER */
#define GPIO_BEEPER_PORT                                                 (GPIOB)

/* Defines for PIN_20: GPIOB.1 with pinCMx 13 on package pin 48 */
#define GPIO_BEEPER_PIN_20_PIN                                   (DL_GPIO_PIN_1)
#define GPIO_BEEPER_PIN_20_IOMUX                                 (IOMUX_PINCM13)
/* Port definition for Pin Group GPIO_LED */
#define GPIO_LED_PORT                                                    (GPIOA)

/* Defines for PIN_8: GPIOA.21 with pinCMx 46 on package pin 17 */
#define GPIO_LED_PIN_8_PIN                                      (DL_GPIO_PIN_21)
#define GPIO_LED_PIN_8_IOMUX                                     (IOMUX_PINCM46)
/* Port definition for Pin Group GPIO_LCD */
#define GPIO_LCD_PORT                                                    (GPIOB)

/* Defines for PIN_RES: GPIOB.10 with pinCMx 27 on package pin 62 */
#define GPIO_LCD_PIN_RES_PIN                                    (DL_GPIO_PIN_10)
#define GPIO_LCD_PIN_RES_IOMUX                                   (IOMUX_PINCM27)
/* Defines for PIN_DC: GPIOB.11 with pinCMx 28 on package pin 63 */
#define GPIO_LCD_PIN_DC_PIN                                     (DL_GPIO_PIN_11)
#define GPIO_LCD_PIN_DC_IOMUX                                    (IOMUX_PINCM28)
/* Defines for PIN_CS: GPIOB.14 with pinCMx 31 on package pin 2 */
#define GPIO_LCD_PIN_CS_PIN                                     (DL_GPIO_PIN_14)
#define GPIO_LCD_PIN_CS_IOMUX                                    (IOMUX_PINCM31)
/* Defines for PIN_BLK: GPIOB.26 with pinCMx 57 on package pin 28 */
#define GPIO_LCD_PIN_BLK_PIN                                    (DL_GPIO_PIN_26)
#define GPIO_LCD_PIN_BLK_IOMUX                                   (IOMUX_PINCM57)
/* Port definition for Pin Group motor_gpio */
#define motor_gpio_PORT                                                  (GPIOA)

/* Defines for STBY: GPIOA.13 with pinCMx 35 on package pin 6 */
#define motor_gpio_STBY_PIN                                     (DL_GPIO_PIN_13)
#define motor_gpio_STBY_IOMUX                                    (IOMUX_PINCM35)
/* Defines for AIN1: GPIOA.15 with pinCMx 37 on package pin 8 */
#define motor_gpio_AIN1_PIN                                     (DL_GPIO_PIN_15)
#define motor_gpio_AIN1_IOMUX                                    (IOMUX_PINCM37)
/* Defines for AIN2: GPIOA.16 with pinCMx 38 on package pin 9 */
#define motor_gpio_AIN2_PIN                                     (DL_GPIO_PIN_16)
#define motor_gpio_AIN2_IOMUX                                    (IOMUX_PINCM38)
/* Defines for BIN1: GPIOA.17 with pinCMx 39 on package pin 10 */
#define motor_gpio_BIN1_PIN                                     (DL_GPIO_PIN_17)
#define motor_gpio_BIN1_IOMUX                                    (IOMUX_PINCM39)
/* Defines for BIN2: GPIOA.18 with pinCMx 40 on package pin 11 */
#define motor_gpio_BIN2_PIN                                     (DL_GPIO_PIN_18)
#define motor_gpio_BIN2_IOMUX                                    (IOMUX_PINCM40)
/* Port definition for Pin Group ENCODERA */
#define ENCODERA_PORT                                                    (GPIOA)

/* Defines for E1A: GPIOA.12 with pinCMx 34 on package pin 5 */
// pins affected by this interrupt request:["E1A"]
#define ENCODERA_INT_IRQN                                       (GPIOA_INT_IRQn)
#define ENCODERA_INT_IIDX                       (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define ENCODERA_E1A_IIDX                                   (DL_GPIO_IIDX_DIO12)
#define ENCODERA_E1A_PIN                                        (DL_GPIO_PIN_12)
#define ENCODERA_E1A_IOMUX                                       (IOMUX_PINCM34)
/* Defines for E1B: GPIOA.31 with pinCMx 6 on package pin 39 */
#define ENCODERA_E1B_PIN                                        (DL_GPIO_PIN_31)
#define ENCODERA_E1B_IOMUX                                        (IOMUX_PINCM6)
/* Port definition for Pin Group ENCODERB */
#define ENCODERB_PORT                                                    (GPIOB)

/* Defines for E2A: GPIOB.12 with pinCMx 29 on package pin 64 */
// pins affected by this interrupt request:["E2A"]
#define ENCODERB_INT_IRQN                                       (GPIOB_INT_IRQn)
#define ENCODERB_INT_IIDX                       (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define ENCODERB_E2A_IIDX                                   (DL_GPIO_IIDX_DIO12)
#define ENCODERB_E2A_PIN                                        (DL_GPIO_PIN_12)
#define ENCODERB_E2A_IOMUX                                       (IOMUX_PINCM29)
/* Defines for E2B: GPIOB.13 with pinCMx 30 on package pin 1 */
#define ENCODERB_E2B_PIN                                        (DL_GPIO_PIN_13)
#define ENCODERB_E2B_IOMUX                                       (IOMUX_PINCM30)
/* Port definition for Pin Group GPIO_IIC */
#define GPIO_IIC_PORT                                                    (GPIOA)

/* Defines for PIN_SCL: GPIOA.2 with pinCMx 7 on package pin 42 */
#define GPIO_IIC_PIN_SCL_PIN                                     (DL_GPIO_PIN_2)
#define GPIO_IIC_PIN_SCL_IOMUX                                    (IOMUX_PINCM7)
/* Defines for PIN_SDA: GPIOA.3 with pinCMx 8 on package pin 43 */
#define GPIO_IIC_PIN_SDA_PIN                                     (DL_GPIO_PIN_3)
#define GPIO_IIC_PIN_SDA_IOMUX                                    (IOMUX_PINCM8)
/* Defines for PIN_0: GPIOB.17 with pinCMx 43 on package pin 14 */
#define track_PIN_0_PORT                                                 (GPIOB)
#define track_PIN_0_PIN                                         (DL_GPIO_PIN_17)
#define track_PIN_0_IOMUX                                        (IOMUX_PINCM43)
/* Defines for PIN_1: GPIOB.18 with pinCMx 44 on package pin 15 */
#define track_PIN_1_PORT                                                 (GPIOB)
#define track_PIN_1_PIN                                         (DL_GPIO_PIN_18)
#define track_PIN_1_IOMUX                                        (IOMUX_PINCM44)
/* Defines for PIN_2: GPIOB.24 with pinCMx 52 on package pin 23 */
#define track_PIN_2_PORT                                                 (GPIOB)
#define track_PIN_2_PIN                                         (DL_GPIO_PIN_24)
#define track_PIN_2_IOMUX                                        (IOMUX_PINCM52)
/* Defines for PIN_3: GPIOB.25 with pinCMx 56 on package pin 27 */
#define track_PIN_3_PORT                                                 (GPIOB)
#define track_PIN_3_PIN                                         (DL_GPIO_PIN_25)
#define track_PIN_3_IOMUX                                        (IOMUX_PINCM56)
/* Defines for PIN_4: GPIOA.22 with pinCMx 47 on package pin 18 */
#define track_PIN_4_PORT                                                 (GPIOA)
#define track_PIN_4_PIN                                         (DL_GPIO_PIN_22)
#define track_PIN_4_IOMUX                                        (IOMUX_PINCM47)
/* Defines for PIN_5: GPIOB.20 with pinCMx 48 on package pin 19 */
#define track_PIN_5_PORT                                                 (GPIOB)
#define track_PIN_5_PIN                                         (DL_GPIO_PIN_20)
#define track_PIN_5_IOMUX                                        (IOMUX_PINCM48)
/* Defines for PIN_6: GPIOA.23 with pinCMx 53 on package pin 24 */
#define track_PIN_6_PORT                                                 (GPIOA)
#define track_PIN_6_PIN                                         (DL_GPIO_PIN_23)
#define track_PIN_6_IOMUX                                        (IOMUX_PINCM53)
/* Defines for PIN_7: GPIOB.22 with pinCMx 50 on package pin 21 */
#define track_PIN_7_PORT                                                 (GPIOB)
#define track_PIN_7_PIN                                         (DL_GPIO_PIN_22)
#define track_PIN_7_IOMUX                                        (IOMUX_PINCM50)
/* Port definition for Pin Group GPIO_KEY */
#define GPIO_KEY_PORT                                                    (GPIOA)

/* Defines for KEY_2: GPIOA.28 with pinCMx 3 on package pin 35 */
#define GPIO_KEY_KEY_2_PIN                                      (DL_GPIO_PIN_28)
#define GPIO_KEY_KEY_2_IOMUX                                      (IOMUX_PINCM3)
/* Defines for KEY_1: GPIOA.8 with pinCMx 19 on package pin 54 */
#define GPIO_KEY_KEY_1_PIN                                       (DL_GPIO_PIN_8)
#define GPIO_KEY_KEY_1_IOMUX                                     (IOMUX_PINCM19)
/* Defines for KEY_3: GPIOA.9 with pinCMx 20 on package pin 55 */
#define GPIO_KEY_KEY_3_PIN                                       (DL_GPIO_PIN_9)
#define GPIO_KEY_KEY_3_IOMUX                                     (IOMUX_PINCM20)
/* Port definition for Pin Group START_KEY */
#define START_KEY_PORT                                                   (GPIOB)

/* Defines for BUTTON: GPIOB.21 with pinCMx 49 on package pin 20 */
#define START_KEY_BUTTON_PIN                                    (DL_GPIO_PIN_21)
#define START_KEY_BUTTON_IOMUX                                   (IOMUX_PINCM49)


/* Defines for MCAN0 */
#define MCAN0_INST                                                        CANFD0
#define GPIO_MCAN0_CAN_TX_PORT                                             GPIOA
#define GPIO_MCAN0_CAN_TX_PIN                                     DL_GPIO_PIN_26
#define GPIO_MCAN0_IOMUX_CAN_TX                                  (IOMUX_PINCM59)
#define GPIO_MCAN0_IOMUX_CAN_TX_FUNC               IOMUX_PINCM59_PF_CANFD0_CANTX
#define GPIO_MCAN0_CAN_RX_PORT                                             GPIOA
#define GPIO_MCAN0_CAN_RX_PIN                                     DL_GPIO_PIN_27
#define GPIO_MCAN0_IOMUX_CAN_RX                                  (IOMUX_PINCM60)
#define GPIO_MCAN0_IOMUX_CAN_RX_FUNC               IOMUX_PINCM60_PF_CANFD0_CANRX
#define MCAN0_INST_IRQHandler                                 CANFD0_IRQHandler
#define MCAN0_INST_INT_IRQN                                     CANFD0_INT_IRQn


/* Defines for MCAN0 MCAN RAM configuration */
#define MCAN0_INST_MCAN_STD_ID_FILT_START_ADDR     (0)
#define MCAN0_INST_MCAN_STD_ID_FILTER_NUM          (0)
#define MCAN0_INST_MCAN_EXT_ID_FILT_START_ADDR     (0)
#define MCAN0_INST_MCAN_EXT_ID_FILTER_NUM          (0)
#define MCAN0_INST_MCAN_TX_BUFF_START_ADDR         (0)
#define MCAN0_INST_MCAN_TX_BUFF_SIZE               (1)
#define MCAN0_INST_MCAN_FIFO_1_START_ADDR          (272)
#define MCAN0_INST_MCAN_FIFO_1_NUM                 (0)
#define MCAN0_INST_MCAN_TX_EVENT_START_ADDR        (16)
#define MCAN0_INST_MCAN_TX_EVENT_SIZE              (0)
#define MCAN0_INST_MCAN_EXT_ID_AND_MASK            (0x1FFFFFFFU)
#define MCAN0_INST_MCAN_RX_BUFF_START_ADDR         (272)
#define MCAN0_INST_MCAN_FIFO_0_START_ADDR          (16)
#define MCAN0_INST_MCAN_FIFO_0_NUM                 (16)

#define MCAN0_INST_MCAN_INTERRUPTS (DL_MCAN_INTERRUPT_BO | \
						DL_MCAN_INTERRUPT_ELO | \
						DL_MCAN_INTERRUPT_EP | \
						DL_MCAN_INTERRUPT_EW | \
						DL_MCAN_INTERRUPT_MRAF | \
						DL_MCAN_INTERRUPT_PEA | \
						DL_MCAN_INTERRUPT_PED | \
						DL_MCAN_INTERRUPT_RF0F | \
						DL_MCAN_INTERRUPT_RF0L | \
						DL_MCAN_INTERRUPT_RF0N | \
						DL_MCAN_INTERRUPT_TC)



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_A_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_TIMER_1_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_WIT_init(void);
void SYSCFG_DL_SPI_LCD_init(void);
void SYSCFG_DL_DMA_init(void);

void SYSCFG_DL_MCAN0_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
