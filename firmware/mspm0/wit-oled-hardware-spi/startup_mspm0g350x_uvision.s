;//*****************************************************************************
;//
;// Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
;//
;// Redistribution and use in source and binary forms, with or without
;// modification, are permitted provided that the following conditions
;// are met:
;//
;//  Redistributions of source code must retain the above copyright
;//  notice, this list of conditions and the following disclaimer.
;//
;//  Redistributions in binary form must reproduce the above copyright
;//  notice, this list of conditions and the following disclaimer in the
;//  documentation and/or other materials provided with the
;//  distribution.
;//
;//  Neither the name of Texas Instruments Incorporated nor the names of
;//  its contributors may be used to endorse or promote products derived
;//  from this software without specific prior written permission.
;//
;// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
;// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
;// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
;// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
;// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
;// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
;// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
;// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
;// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
;// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
;// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;//
;// MSPM0G3507 startup file. The stack is enlarged for the 1 ms CAN ISR.
;//
;//****************************************************************************

; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp
                DCD     Reset_Handler
                DCD     NMI_Handler
                DCD     HardFault_Handler
                DCD     0
                DCD     0
                DCD     0
                DCD     0
                DCD     0
                DCD     0
                DCD     0
                DCD     SVC_Handler
                DCD     0
                DCD     0
                DCD     PendSV_Handler
                DCD     SysTick_Handler

                ; External Interrupts
                DCD     GROUP0_IRQHandler
                DCD     GROUP1_IRQHandler
                DCD     TIMG8_IRQHandler
                DCD     UART3_IRQHandler
                DCD     ADC0_IRQHandler
                DCD     ADC1_IRQHandler
                DCD     CANFD0_IRQHandler
                DCD     DAC0_IRQHandler
                DCD     0
                DCD     SPI0_IRQHandler
                DCD     SPI1_IRQHandler
                DCD     0
                DCD     0
                DCD     UART1_IRQHandler
                DCD     UART2_IRQHandler
                DCD     UART0_IRQHandler
                DCD     TIMG0_IRQHandler
                DCD     TIMG6_IRQHandler
                DCD     TIMA0_IRQHandler
                DCD     TIMA1_IRQHandler
                DCD     TIMG7_IRQHandler
                DCD     TIMG12_IRQHandler
                DCD     0
                DCD     0
                DCD     I2C0_IRQHandler
                DCD     I2C1_IRQHandler
                DCD     0
                DCD     0
                DCD     AES_IRQHandler
                DCD     0
                DCD     RTC_IRQHandler
                DCD     DMA_IRQHandler

__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  __main
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC
                EXPORT  Default_Handler           [WEAK]
                EXPORT  GROUP0_IRQHandler         [WEAK]
                EXPORT  GROUP1_IRQHandler         [WEAK]
                EXPORT  TIMG8_IRQHandler          [WEAK]
                EXPORT  UART3_IRQHandler          [WEAK]
                EXPORT  ADC0_IRQHandler           [WEAK]
                EXPORT  ADC1_IRQHandler           [WEAK]
                EXPORT  CANFD0_IRQHandler         [WEAK]
                EXPORT  DAC0_IRQHandler           [WEAK]
                EXPORT  SPI0_IRQHandler           [WEAK]
                EXPORT  SPI1_IRQHandler           [WEAK]
                EXPORT  UART1_IRQHandler          [WEAK]
                EXPORT  UART2_IRQHandler          [WEAK]
                EXPORT  UART0_IRQHandler          [WEAK]
                EXPORT  TIMG0_IRQHandler          [WEAK]
                EXPORT  TIMG6_IRQHandler          [WEAK]
                EXPORT  TIMA0_IRQHandler          [WEAK]
                EXPORT  TIMA1_IRQHandler          [WEAK]
                EXPORT  TIMG7_IRQHandler          [WEAK]
                EXPORT  TIMG12_IRQHandler         [WEAK]
                EXPORT  I2C0_IRQHandler           [WEAK]
                EXPORT  I2C1_IRQHandler           [WEAK]
                EXPORT  AES_IRQHandler            [WEAK]
                EXPORT  RTC_IRQHandler            [WEAK]
                EXPORT  DMA_IRQHandler            [WEAK]

GROUP0_IRQHandler
GROUP1_IRQHandler
TIMG8_IRQHandler
UART3_IRQHandler
ADC0_IRQHandler
ADC1_IRQHandler
CANFD0_IRQHandler
DAC0_IRQHandler
SPI0_IRQHandler
SPI1_IRQHandler
UART1_IRQHandler
UART2_IRQHandler
UART0_IRQHandler
TIMG0_IRQHandler
TIMG6_IRQHandler
TIMA0_IRQHandler
TIMA1_IRQHandler
TIMG7_IRQHandler
TIMG12_IRQHandler
I2C0_IRQHandler
I2C1_IRQHandler
AES_IRQHandler
RTC_IRQHandler
DMA_IRQHandler
                B       .
                ENDP

                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF

                END
