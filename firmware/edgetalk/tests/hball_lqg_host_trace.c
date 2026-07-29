#include "hball_lqg.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    hball_lqg_t controller;
    hball_lqg_input_t input;
    float command = 0.0F;
    unsigned int index;

    hball_lqg_init(&controller, 0.004F);
    memset(&input, 0, sizeof(input));
    input.lateral_accel_mps2 = -0.04F;
    input.yaw_rate_rad_s = 0.35F;

    for (index = 0U; index < 20U; ++index)
    {
        input.beam_angle_rad = 0.01F * sinf(0.1F * (float)index);
        input.body_pitch_rad = 0.002F * cosf(0.07F * (float)index);
        input.longitudinal_accel_mps2 = 0.10F + 0.002F * (float)index;
        hball_lqg_predict(&controller, 0.005F, &input);
        if ((index % 3U) == 0U)
        {
            (void)hball_lqg_update_delayed_position(
                &controller, 0.004F + 0.0001F * (float)index, 0.033F
            );
        }
        command = hball_lqg_command(&controller, &input, 0.0F);
    }

    printf(
        "TRACE %.9f %.9f %.9f %.9f %lu %lu\n",
        (double)controller.state[0],
        (double)controller.state[1],
        (double)controller.state[2],
        (double)command,
        (unsigned long)controller.accepted_camera_updates,
        (unsigned long)controller.rejected_camera_updates
    );
    return 0;
}
