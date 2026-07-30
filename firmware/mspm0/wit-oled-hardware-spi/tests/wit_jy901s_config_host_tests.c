#include "wit_jy901s_config.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

static void assert_command(
    const uint8_t actual[WIT_JY901S_COMMAND_SIZE],
    const uint8_t expected[WIT_JY901S_COMMAND_SIZE])
{
    for (size_t i = 0U; i < WIT_JY901S_COMMAND_SIZE; ++i)
    {
        assert(actual[i] == expected[i]);
    }
}

int main(void)
{
    uint8_t command[WIT_JY901S_COMMAND_SIZE];
    const uint8_t unlock[WIT_JY901S_COMMAND_SIZE] = {
        0xFFU, 0xAAU, 0x69U, 0x88U, 0xB5U};
    const uint8_t enable_control_reports[WIT_JY901S_COMMAND_SIZE] = {
        0xFFU, 0xAAU, 0x02U, 0x0EU, 0x00U};

    wit_jy901s_build_write_command(
        WIT_JY901S_REG_KEY, WIT_JY901S_KEY_UNLOCK, command);
    assert_command(command, unlock);

    wit_jy901s_build_write_command(
        WIT_JY901S_REG_OUTPUT_CONTENT,
        WIT_JY901S_OUTPUT_ACCEL_GYRO_ANGLE,
        command);
    assert_command(command, enable_control_reports);
    return 0;
}
