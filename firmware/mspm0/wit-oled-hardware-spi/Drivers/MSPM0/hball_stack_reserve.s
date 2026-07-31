; The TI MSPM0G3507 Keil startup contributes the final 0x100-byte STACK
; area and defines __initial_sp at its end. Link this 0x700-byte extension
; before the vendor startup object so the effective downward-growing stack
; is 0x800 bytes without modifying the installed SDK.

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
                EXPORT  hball_stack_extension
hball_stack_extension
                SPACE   0x00000700

                END
