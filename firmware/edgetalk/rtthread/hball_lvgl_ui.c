#include "hball_m55_ui.h"

#include <lvgl.h>

#define HBALL_UI_REFRESH_MS 100U

typedef struct
{
    lv_obj_t *status;
    lv_obj_t *ball_position;
    lv_obj_t *ball_velocity;
    lv_obj_t *imu_accel;
    lv_obj_t *yaw_rate;
    lv_obj_t *motor_angle;
    lv_obj_t *lqg_target;
    lv_obj_t *can_rx;
    lv_obj_t *vision_age;
} hball_ui_widgets_t;

static hball_ui_widgets_t g_hball_ui;

static lv_obj_t *hball_ui_make_metric(
    lv_obj_t *parent, const char *title, int column, int row
)
{
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_t *name = lv_label_create(card);
    lv_obj_t *value = lv_label_create(card);

    lv_obj_set_grid_cell(
        card,
        LV_GRID_ALIGN_STRETCH,
        column,
        1,
        LV_GRID_ALIGN_STRETCH,
        row,
        1
    );
    lv_obj_set_style_bg_color(card, lv_color_hex(0x172033), 0);
    lv_obj_set_style_border_color(card, lv_color_hex(0x2B3A55), 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_radius(card, 12, 0);
    lv_obj_set_style_pad_all(card, 12, 0);

    lv_label_set_text(name, title);
    lv_obj_set_style_text_color(name, lv_color_hex(0x8EA3C0), 0);
    lv_obj_align(name, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_label_set_text(value, "--");
    lv_obj_set_style_text_color(value, lv_color_hex(0xF5F8FF), 0);
    lv_obj_align(value, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    return value;
}

static void hball_ui_update(lv_timer_t *timer)
{
    hball_m55_ui_snapshot_t snapshot;

    LV_UNUSED(timer);
    hball_m55_get_ui_snapshot(&snapshot);
    lv_label_set_text_fmt(
        g_hball_ui.ball_position,
        "%ld um",
        (long)(snapshot.ball_position_m * 1000000.0F)
    );
    lv_label_set_text_fmt(
        g_hball_ui.ball_velocity,
        "%ld um/s",
        (long)(snapshot.ball_velocity_mps * 1000000.0F)
    );
    if ((snapshot.valid_flags & HBALL_UI_VALID_IMU) != 0U)
    {
        lv_label_set_text_fmt(
            g_hball_ui.imu_accel,
            "%ld mm/s2",
            (long)(snapshot.longitudinal_accel_mps2 * 1000.0F)
        );
        lv_label_set_text_fmt(
            g_hball_ui.yaw_rate,
            "%ld mrad/s",
            (long)(snapshot.yaw_rate_rad_s * 1000.0F)
        );
    }
    else
    {
        lv_label_set_text(g_hball_ui.imu_accel, "WAIT");
        lv_label_set_text(g_hball_ui.yaw_rate, "WAIT");
    }
    if ((snapshot.valid_flags & HBALL_UI_VALID_MOTOR) != 0U)
    {
        lv_label_set_text_fmt(
            g_hball_ui.motor_angle,
            "%ld mrad",
            (long)(snapshot.motor_angle_rad * 1000.0F)
        );
    }
    else
    {
        lv_label_set_text(g_hball_ui.motor_angle, "WAIT");
    }
    lv_label_set_text_fmt(
        g_hball_ui.lqg_target,
        "%ld mrad",
        (long)(snapshot.lqg_target_rad * 1000.0F)
    );
    lv_label_set_text_fmt(
        g_hball_ui.can_rx, "%lu", (unsigned long)snapshot.can_rx_total
    );
    if ((snapshot.valid_flags & HBALL_UI_VALID_VISION) != 0U)
    {
        lv_label_set_text_fmt(
            g_hball_ui.vision_age,
            "%lu ms",
            (unsigned long)snapshot.vision_age_ms
        );
    }
    else
    {
        lv_label_set_text(g_hball_ui.vision_age, "WAIT");
    }
    lv_label_set_text_fmt(
        g_hball_ui.status,
        "SHADOW / TX OFF  |  200 Hz  |  steps %lu  miss %lu",
        (unsigned long)snapshot.controller_steps,
        (unsigned long)snapshot.deadline_misses
    );
}

void lv_user_gui_init(void)
{
    static int32_t columns[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static int32_t rows[] = {
        LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1),
        LV_GRID_TEMPLATE_LAST
    };
    lv_obj_t *screen = lv_screen_active();
    lv_obj_t *title = lv_label_create(screen);
    lv_obj_t *grid = lv_obj_create(screen);

    lv_obj_set_style_bg_color(screen, lv_color_hex(0x0B1020), 0);
    lv_obj_set_style_text_color(screen, lv_color_hex(0xF5F8FF), 0);

    lv_label_set_text(title, "H-BALL CONTROL / EDGETALK M55");
    lv_obj_set_style_text_color(title, lv_color_hex(0x5EEAD4), 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 18, 16);

    g_hball_ui.status = lv_label_create(screen);
    lv_label_set_text(g_hball_ui.status, "SHADOW / TX OFF");
    lv_obj_set_style_text_color(g_hball_ui.status, lv_color_hex(0xFBBF24), 0);
    lv_obj_align(g_hball_ui.status, LV_ALIGN_TOP_LEFT, 18, 48);

    lv_obj_set_size(grid, lv_pct(100), lv_pct(82));
    lv_obj_align(grid, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_grid_dsc_array(grid, columns, rows);
    lv_obj_set_layout(grid, LV_LAYOUT_GRID);
    lv_obj_set_style_bg_opa(grid, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(grid, 0, 0);
    lv_obj_set_style_pad_all(grid, 12, 0);
    lv_obj_set_style_pad_row(grid, 10, 0);
    lv_obj_set_style_pad_column(grid, 10, 0);

    g_hball_ui.ball_position = hball_ui_make_metric(grid, "BALL POSITION", 0, 0);
    g_hball_ui.ball_velocity = hball_ui_make_metric(grid, "BALL VELOCITY", 1, 0);
    g_hball_ui.imu_accel = hball_ui_make_metric(grid, "IMU ACCEL", 0, 1);
    g_hball_ui.yaw_rate = hball_ui_make_metric(grid, "YAW RATE", 1, 1);
    g_hball_ui.motor_angle = hball_ui_make_metric(grid, "MOTOR ANGLE", 0, 2);
    g_hball_ui.lqg_target = hball_ui_make_metric(grid, "LQG TARGET", 1, 2);
    g_hball_ui.can_rx = hball_ui_make_metric(grid, "CAN RX", 0, 3);
    g_hball_ui.vision_age = hball_ui_make_metric(grid, "VISION AGE", 1, 3);

    (void)lv_timer_create(hball_ui_update, HBALL_UI_REFRESH_MS, NULL);
    hball_ui_update(NULL);
}
