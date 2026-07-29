/* Include inside SECTIONS, before every other m33_m55_shared output section. */
.hball_ipc_shared ORIGIN(m33_m55_shared) (NOLOAD) :
{
    . = ALIGN(32);
    __hball_ipc_shared_start__ = .;
    KEEP(*(.hball_ipc_shared))
    . = ALIGN(32);
    __hball_ipc_shared_end__ = .;
} > m33_m55_shared

ASSERT(
    ADDR(.hball_ipc_shared) == ORIGIN(m33_m55_shared),
    "H-ball IPC must start at shared SRAM origin"
)
ASSERT(
    SIZEOF(.hball_ipc_shared) == 0x100,
    "H-ball IPC ABI must reserve exactly 256 bytes"
)
