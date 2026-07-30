#include "hball_deployment_controller.h"
#include "hball_fourbar.h"

#include <assert.h>
#include <math.h>
#include <string.h>

#define DEG_TO_RAD(value) ((value) * 0.01745329251994329577F)

static void test_fourbar_level_and_limits_round_trip(void)
{
    hball_fourbar_geometry_t geometry;
    const float angles[3] = {
        DEG_TO_RAD(-6.0F), 0.0F, DEG_TO_RAD(6.0F)
    };
    float previous_offset = -10.0F;

    hball_fourbar_default_geometry(&geometry);
    for (unsigned int index = 0U; index < 3U; ++index)
    {
        float geometric_motor_angle;
        float recovered_pipe_angle;
        float motor_offset;

        assert(hball_fourbar_inverse(
            &geometry, angles[index], &geometric_motor_angle
        ));
        assert(hball_fourbar_forward(
            &geometry, geometric_motor_angle, &recovered_pipe_angle
        ));
        assert(hball_fourbar_motor_offset(
            &geometry, angles[index], &motor_offset
        ));
        assert(fabsf(recovered_pipe_angle - angles[index]) < 2.0e-5F);
        assert(motor_offset > previous_offset);
        previous_offset = motor_offset;
    }
    assert(fabsf(previous_offset - DEG_TO_RAD(70.68F)) < DEG_TO_RAD(0.2F));
}

static void test_history_update_replays_to_current_state(void)
{
    hball_deployment_controller_t controller;
    hball_deployment_input_t input;

    hball_deployment_controller_init(&controller, 0.020F);
    memset(&input, 0, sizeof(input));
    for (unsigned int index = 0U; index < 20U; ++index)
    {
        hball_deployment_controller_predict(&controller, 0.005F, &input);
    }
    assert(hball_deployment_controller_update_delayed_position(
        &controller, 0.010F, 0.040F
    ));
    assert(controller.accepted_camera_updates == 1U);
    assert(controller.history_count == 20U);
    assert(isfinite(controller.state[0]));
    assert(isfinite(controller.state[1]));
    assert(!hball_deployment_controller_update_delayed_position(
        &controller, 0.0F, 0.500F
    ));
    assert(controller.too_old_camera_updates == 1U);
}

static void test_lqi_sign_rate_limit_and_edge_recovery(void)
{
    hball_deployment_controller_t controller;
    hball_deployment_input_t input;
    float command;

    hball_deployment_controller_init(&controller, 0.020F);
    memset(&input, 0, sizeof(input));
    command = hball_deployment_controller_command(
        &controller, &input, 0.0F, 0.005F, true
    );
    assert(command < 0.0F);
    assert(fabsf(command) <= 0.001751F);

    controller.state[0] = 0.090F;
    controller.state[1] = 0.20F;
    command = hball_deployment_controller_command(
        &controller, &input, 0.0F, 0.005F, true
    );
    assert(command < 0.0F);
    assert(controller.edge_recovery_total == 1U);
    assert(fabsf(command) <= DEG_TO_RAD(6.0F));
}

int main(void)
{
    test_fourbar_level_and_limits_round_trip();
    test_history_update_replays_to_current_state();
    test_lqi_sign_rate_limit_and_edge_recovery();
    return 0;
}
