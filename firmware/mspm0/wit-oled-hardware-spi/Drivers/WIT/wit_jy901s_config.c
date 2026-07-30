#include "wit_jy901s_config.h"

#include <stddef.h>

void wit_jy901s_build_write_command(
    uint8_t register_address,
    uint16_t value,
    uint8_t command[WIT_JY901S_COMMAND_SIZE])
{
    if (command == NULL)
    {
        return;
    }
    command[0] = 0xFFU;
    command[1] = 0xAAU;
    command[2] = register_address;
    command[3] = (uint8_t)(value & 0x00FFU);
    command[4] = (uint8_t)(value >> 8U);
}
