/*
 * Copyright (c) 2021, Texas Instruments Incorporated
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

#include "ti_msp_dl_config.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "hball_can_port.h"
#include "hball_mission_run_guard.h"
#include "hball_mission_policy.h"
#include "hball_runtime_services.h"
#include "hball_runtime_target.h"
#include "chassis_motion_profile.h"
#include "line_follower.h"
#include "line_sensor_port.h"
#include "line_snapshot.h"
#include "motion_intent.h"
#include "route_marker_detector.h"
#include "wheel_control.h"

/*
 * There is no process command line on the target. Avoid Arm C library
 * semihosted argv startup, which otherwise stops at BKPT 0xAB after reset.
 */
__asm(".global __ARM_use_no_argv\n");

#define APP_MODE_CAR             0U
#define APP_MODE_GYRO_LCD_TEST   1U
#define APP_MODE_LCD_TEST        2U
#define APP_MODE_MOTOR_TEST      3U
#define APP_MODE_ENCODER_TEST    4U
#define APP_MODE_TRACK_TEST      5U
#define APP_MODE_TRACK_MOTOR_TEST 6U
#define APP_MODE_TRACK_GROUND_TEST 7U
#define APP_MODE_LAP_TEST        8U
#define APP_MODE_SPEED_CAL_TEST  9U
#define APP_MODE_SPEED_PI_TEST   10U
#define APP_MODE_PWM_SWEEP_TEST  11U
#define APP_MODE_MOTOR_MAP_TEST  12U
#define APP_MODE                 APP_MODE_LAP_TEST
#define HBALL_MISSION_LOCAL_MOTION_ENABLED 1U

_Static_assert(
    HBALL_MISSION_LOCAL_MOTION_ENABLED == 1U,
    "competition mission menu must control local motion"
);

#define CAR_TASK_LAP_STOP        1U
#define CAR_TASK_TIMED_RUN       2U
#define CAR_TASK_STABLE_LAP      3U
#define GYRO_LCD_REFRESH_MS 100U
#define HBALL_MISSION_MENU_RENDER_MIN_MS 1000U
#define HBALL_Q4_SOFT_START_MS 600U
#define HBALL_Q4_SOFT_STOP_MS 800U
#define HBALL_Q4_DUTY_SLEW_STEP 5
#define HBALL_Q56_SOFT_STOP_MS 900U
#define HBALL_Q56_STOP_SPEED_THRESHOLD 2
#define HBALL_Q56_STOP_SETTLE_MS 300U
#define HBALL_Q56_WHEEL_SAMPLE_STALE_MS (WHEEL_CONTROL_PERIOD_MS * 2U)
#define HBALL_Q56_STOP_FALLBACK_MS 600U
#define MOTOR_TEST_DUTY 20.0f

uint8_t oled_buffer[64];
uint8_t uart_send[64];
void ui_home_page(void);// 首页页面初始化
static void lcd_self_test(void);
static void gyro_lcd_screen_init(void);
static void gyro_lcd_test(void);
static void format_angle(char label, float angle, char text[10]);
static void motor_test(void);
static void motor_test_step(const char *label, float left_pwm, float right_pwm);
static void encoder_test(void);
static void format_encoder_count(char label, int32_t count, char text[10]);
static void format_edge_count(char encoder, char phase, uint32_t count, char text[10]);
static void track_sensor_test(void);
static void format_track_error(int16_t error, char text[7]);
static void track_motor_test(void);
static void track_ground_test(void);
static void lap_test(void);
static void lap_test_once(void);
static void competition_runtime_wait_ms(uint32_t duration_ms);
static void mission_lcd_fill_serviced(
    unsigned int xsta,
    unsigned int ysta,
    unsigned int xend,
    unsigned int yend,
    unsigned int color
);
static uint8_t select_car_task(void);
static void render_mission_menu(
    const hball_mission_menu_view_t *view
);
static void format_hex16(uint16_t value, char text[5]);
static void format_lap_time(uint32_t elapsed_ms, char text[8]);
static void speed_calibration_test(void);
static void telemetry_send_string(const char *text);
static uint8_t speed_calibration_start_requested(void);
static void speed_pi_test(void);
static void pwm_sweep_test(void);
static void motor_encoder_map_test(void);

void uart1_send_char(char ch)
{
    // 当串口忙的时候等待，不忙时发送字符
    while( DL_UART_isBusy(UART_1_INST) == true );
    DL_UART_Main_transmitData(UART_1_INST, ch);
}

//串口发送字符串
void uart1_send_string(char* str)
{
    while(str != NULL && *str != '\0')
    {
        uart1_send_char(*str++);
    }
}

static void telemetry_send_string(const char *text)
{
    while ((text != NULL) && (*text != '\0'))
    {
        while (DL_UART_isBusy(UART_1_INST) == true);
        DL_UART_Main_transmitData(UART_1_INST, (uint8_t)*text);
        while (DL_UART_isBusy(UART_0_INST) == true);
        DL_UART_Main_transmitData(UART_0_INST, (uint8_t)*text);
        text++;
    }
}

static uint8_t speed_calibration_start_requested(void)
{
    uint8_t received;

    if (DL_GPIO_readPins(START_KEY_PORT, START_KEY_BUTTON_PIN) == 0U) {
        return 1U;
    }

    if (DL_UART_isRXFIFOEmpty(UART_1_INST) == false)
    {
        received = DL_UART_Main_receiveData(UART_1_INST);
        if ((received == 'G') || (received == 'g')) {
            return 1U;
        }
    }

    if (DL_UART_isRXFIFOEmpty(UART_0_INST) == false)
    {
        received = DL_UART_Main_receiveData(UART_0_INST);
        if ((received == 'G') || (received == 'g')) {
            return 1U;
        }
    }

    return 0U;
}

int key_num=0;
int round_number=1;
volatile int start=0;
int quetion_num=0;
float now_yaw=0;
char huidu_char;

extern PID_t LEFT;
extern PID_t RIGHT;
volatile int32_t Get_Encoder_countA=0;
volatile int32_t Get_Encoder_countB=0;
int32_t encoderA_cnt=0;
int32_t encoderB_cnt=0;
int32_t Get_Encoder_countA_LAST=0;
int32_t Get_Encoder_countB_LAST=0;


int main(void){
    SYSCFG_DL_init();
    /*
     * Several APP_MODE handlers intentionally never return. Start the WIT
     * UART/DMA before dispatching to them so CAN telemetry carries real
     * source samples instead of a 200 Hz mirror of zero-initialized data.
     */
    WIT_Init();
    hball_can_port_init();
    hball_runtime_target_init();
#if (APP_MODE != APP_MODE_GYRO_LCD_TEST) && (APP_MODE != APP_MODE_ENCODER_TEST)
    SysTick_Init();
#endif

#if APP_MODE == APP_MODE_LCD_TEST
    chassis_actuator_stop();
    chassis_actuator_disable();
    lcd_init();
    lcd_self_test();
#elif APP_MODE == APP_MODE_GYRO_LCD_TEST
    /*
     * 独立测试模式只启用 LCD 和 WIT。电机驱动保持待机，避免桌面测试时
     * 车轮意外动作。把 APP_MODE 改为 APP_MODE_CAR 即可恢复小车程序。
     */
    chassis_actuator_stop();
    chassis_actuator_disable();
    lcd_init();
    gyro_lcd_screen_init();
    gyro_lcd_test();
#elif APP_MODE == APP_MODE_MOTOR_TEST
    chassis_actuator_stop();
    chassis_actuator_disable();
    lcd_init();
    motor_test();
#elif APP_MODE == APP_MODE_ENCODER_TEST
    chassis_actuator_stop();
    chassis_actuator_disable();
    lcd_init();
    encoder_test();
#elif APP_MODE == APP_MODE_TRACK_TEST
    chassis_actuator_stop();
    chassis_actuator_disable();
    lcd_init();
    track_sensor_test();
#elif APP_MODE == APP_MODE_TRACK_MOTOR_TEST
    chassis_actuator_stop();
    chassis_actuator_disable();
    lcd_init();
    track_motor_test();
#elif APP_MODE == APP_MODE_TRACK_GROUND_TEST
    chassis_actuator_stop();
    chassis_actuator_disable();
    lcd_init();
    track_ground_test();
#elif APP_MODE == APP_MODE_LAP_TEST
    chassis_actuator_stop();
    chassis_actuator_disable();
    lcd_init();
    lap_test();
#elif APP_MODE == APP_MODE_SPEED_CAL_TEST
    chassis_actuator_stop();
    chassis_actuator_disable();
    lcd_init();
    speed_calibration_test();
#elif APP_MODE == APP_MODE_SPEED_PI_TEST
    chassis_actuator_stop();
    chassis_actuator_disable();
    lcd_init();
    speed_pi_test();
#elif APP_MODE == APP_MODE_PWM_SWEEP_TEST
    chassis_actuator_stop();
    chassis_actuator_disable();
    lcd_init();
    pwm_sweep_test();
#elif APP_MODE == APP_MODE_MOTOR_MAP_TEST
    chassis_actuator_stop();
    chassis_actuator_disable();
    lcd_init();
    motor_encoder_map_test();
#endif
    
    // 初始化串口用于 printf 重定向
    Bluetooth_Init();
    lcd_init();
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    Interrupt_Init();
    ui_home_page();

    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    chassis_actuator_init();

    // 将STBY置为高电平
    chassis_actuator_enable();

    LCD_ShowString(0, 0, (const unsigned char *)"the_car_is_ready", BLUE, WHITE, 32, 0);

    while (1)
    {
      if (start == 0)
      {
        key_num = get_keynum();

        if (key_num == 1) // 寻迹圈数选择
        {
            LCD_ShowString(0, 0, (const unsigned char *)"                   ", BLUE, BLACK, 32, 0);
            LCD_ShowString(0, 0, (const unsigned char *)"cycle:", BLUE, WHITE, 32, 0);
            // 限制最大圈数
            beep();
            round_number++;
            if (round_number > 5)
            {
              round_number = 1;
            }
            LCD_ShowIntNum(100, 0, round_number, 3, BLUE, WHITE, 32);         
        }
        else if (key_num == 2) // 视觉题目选择
        {
            LCD_ShowString(0, 0, (const unsigned char *)"                   ", BLUE, BLACK, 32, 0);            
            LCD_ShowString(0, 0, (const unsigned char *)"quetion:", BLUE, WHITE, 32, 0);
            beep();
            quetion_num++;
            if (quetion_num > 3)
            {
            quetion_num = 0;
            }
            LCD_ShowIntNum(150, 0, quetion_num, 3, BLUE, WHITE, 32);
        }
        else if (key_num == 3) // 启动
        {
            LCD_ShowString(0, 0, (const unsigned char *)"                   ", BLUE, BLACK, 32, 0);
            if (quetion_num == 0)
            {
                track_start((uint8_t)round_number);
                start = 1;
            }
            else
            {
                LCD_ShowString(0, 0, (const unsigned char *)"vision_not_ready", BLUE, WHITE, 32, 0);
                beep();
            }
        }
        delay_cycles(100);
      }
      else if (start != 0 && quetion_num==0)
      {
        my_track();
      }
    }
}

static void lcd_self_test(void)
{
    LCD_BLK_Set();
    LCD_Fill(0, 0, 105, LCD_H - 1U, RED);
    LCD_Fill(106, 0, 212, LCD_H - 1U, GREEN);
    LCD_Fill(213, 0, LCD_W - 1U, LCD_H - 1U, BLUE);
    LCD_Fill(88, 58, 232, 110, BLACK);
    LCD_ShowString(100, 72, (const unsigned char *)"LCD OK", WHITE, BLACK, 24, 0);

    while (1)
    {
        DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_PIN_8_PIN);
        mspm0_delay_ms(500);
    }
}

static void format_angle(char label, float angle, char text[10])
{
    uint16_t magnitude;

    if (angle > 180.0f) {
        angle = 180.0f;
    } else if (angle < -180.0f) {
        angle = -180.0f;
    }

    text[0] = label;
    text[1] = ':';
    text[2] = (angle < 0.0f) ? '-' : '+';
    if (angle < 0.0f) {
        angle = -angle;
    }

    magnitude = (uint16_t)(angle * 10.0f + 0.5f);
    text[3] = (char)('0' + ((magnitude / 1000U) % 10U));
    text[4] = (char)('0' + ((magnitude / 100U) % 10U));
    text[5] = (char)('0' + ((magnitude / 10U) % 10U));
    text[6] = '.';
    text[7] = (char)('0' + (magnitude % 10U));
    text[8] = '\0';
}

static void gyro_lcd_screen_init(void)
{
    LCD_BLK_Set();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_ShowString(4, 4, (const unsigned char *)"WIT", WHITE, BLACK, 24, 0);
    LCD_ShowString(4, 36, (const unsigned char *)"1:+000.0", WHITE, BLACK, 24, 0);
    LCD_ShowString(4, 68, (const unsigned char *)"2:+000.0", WHITE, BLACK, 24, 0);
    LCD_ShowString(4, 100, (const unsigned char *)"3:+000.0", CYAN, BLACK, 24, 0);
    LCD_Fill(0, 132, 120, LCD_H - 1U, RED);
    LCD_ShowString(4, 138, (const unsigned char *)"WAIT", WHITE, RED, 24, 0);
}

static void gyro_lcd_test(void)
{
    uint32_t rx_byte_count;
    uint32_t valid_frame_count;
    uint32_t frame_count;
    uint32_t last_rx_byte_count = 0U;
    uint32_t last_valid_frame_count = 0U;
    uint32_t last_frame_count = 0U;
    float roll;
    float pitch;
    float yaw;
    char angle_text[10];

    while (1)
    {
        __disable_irq();
        roll = wit_data.roll;
        pitch = wit_data.pitch;
        yaw = wit_data.yaw;
        rx_byte_count = wit_rx_byte_count;
        valid_frame_count = wit_valid_frame_count;
        frame_count = wit_angle_frame_count;
        __enable_irq();

        /*
         * Do not continuously rewrite the LCD when the UART is silent.
         * Besides avoiding unnecessary SPI traffic, this makes the red WAIT
         * screen a stable and unambiguous "no RX data" indication.
         */
        if ((rx_byte_count == last_rx_byte_count) &&
            (valid_frame_count == last_valid_frame_count) &&
            (frame_count == last_frame_count))
        {
            delay_cycles(CPUCLK_FREQ / 1000U);
            continue;
        }

        if (frame_count != last_frame_count)
        {
            format_angle('1', roll, angle_text);
            LCD_ShowString(4, 36, (const unsigned char *)angle_text, WHITE, BLACK, 24, 0);
            format_angle('2', pitch, angle_text);
            LCD_ShowString(4, 68, (const unsigned char *)angle_text, WHITE, BLACK, 24, 0);
            format_angle('3', yaw, angle_text);
            LCD_ShowString(4, 100, (const unsigned char *)angle_text, CYAN, BLACK, 24, 0);
        }

        if (frame_count > 0U)
        {
            LCD_Fill(0, 132, 120, LCD_H - 1U, GREEN);
            LCD_ShowString(4, 138, (const unsigned char *)"3", BLACK, GREEN, 24, 0);
        }
        else if (valid_frame_count > 0U)
        {
            LCD_Fill(0, 132, 120, LCD_H - 1U, CYAN);
            LCD_ShowString(4, 138, (const unsigned char *)"2", BLACK, CYAN, 24, 0);
        }
        else if (rx_byte_count > 0U)
        {
            LCD_Fill(0, 132, 120, LCD_H - 1U, YELLOW);
            LCD_ShowString(4, 138, (const unsigned char *)"1", BLACK, YELLOW, 24, 0);
        }

        last_rx_byte_count = rx_byte_count;
        last_valid_frame_count = valid_frame_count;
        last_frame_count = frame_count;
        delay_cycles(CPUCLK_FREQ / (1000U / GYRO_LCD_REFRESH_MS));
    }
}

static void motor_test_step(const char *label, float left_pwm, float right_pwm)
{
    LCD_Fill(0, 64, 180, 104, BLACK);
    LCD_ShowString(4, 70, (const unsigned char *)label, YELLOW, BLACK, 32, 0);

    chassis_actuator_set_pwm(left_pwm, right_pwm);
    delay_cycles(CPUCLK_FREQ * 2U);

    chassis_actuator_stop();
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
    delay_cycles(CPUCLK_FREQ);
}

static void motor_test(void)
{
    chassis_actuator_init();
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);

    LCD_BLK_Set();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_ShowString(4, 4, (const unsigned char *)"MOTOR", WHITE, BLACK, 32, 0);
    LCD_ShowString(4, 38, (const unsigned char *)"20", CYAN, BLACK, 24, 0);

    chassis_actuator_enable();

    /*
     * Calibrated from the individual-wheel test:
     * left forward is negative, right forward is positive.
     */
    motor_test_step("FWD", -MOTOR_TEST_DUTY, MOTOR_TEST_DUTY);
    motor_test_step("REV", MOTOR_TEST_DUTY, -MOTOR_TEST_DUTY);

    chassis_actuator_stop();
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
    chassis_actuator_disable();

    LCD_Fill(0, 64, 180, 104, BLACK);
    LCD_ShowString(4, 70, (const unsigned char *)"END", GREEN, BLACK, 32, 0);

    while (1)
    {
        __WFI();
    }
}

static void format_encoder_count(char label, int32_t count, char text[10])
{
    uint32_t magnitude;

    if (count > 99999) {
        count = 99999;
    } else if (count < -99999) {
        count = -99999;
    }

    text[0] = label;
    text[1] = ':';
    text[2] = (count < 0) ? '-' : '+';
    magnitude = (count < 0) ? (uint32_t)(-count) : (uint32_t)count;
    text[3] = (char)('0' + ((magnitude / 10000U) % 10U));
    text[4] = (char)('0' + ((magnitude / 1000U) % 10U));
    text[5] = (char)('0' + ((magnitude / 100U) % 10U));
    text[6] = (char)('0' + ((magnitude / 10U) % 10U));
    text[7] = (char)('0' + (magnitude % 10U));
    text[8] = '\0';
}

static void encoder_test(void)
{
    uint32_t e1a;
    uint32_t e2a;
    uint32_t last_e1a = UINT32_MAX;
    uint32_t last_e2a = UINT32_MAX;
    char edge_text[10];

    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    encoder_e1a_edges = 0;
    encoder_e1b_edges = 0;
    encoder_e2a_edges = 0;
    encoder_e2b_edges = 0;

    LCD_BLK_Set();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_ShowString(4, 2, (const unsigned char *)"A ONLY", WHITE, BLACK, 24, 0);

    DL_GPIO_clearInterruptStatus(ENCODERA_PORT, ENCODERA_E1A_PIN);
    DL_GPIO_clearInterruptStatus(ENCODERB_PORT, ENCODERB_E2A_PIN);
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);

    while (1)
    {
        __disable_irq();
        e1a = encoder_e1a_edges;
        e2a = encoder_e2a_edges;
        __enable_irq();

        if (e1a != last_e1a)
        {
            format_edge_count('1', 'A', e1a, edge_text);
            LCD_ShowString(4, 48, (const unsigned char *)edge_text, CYAN, BLACK, 32, 0);
            last_e1a = e1a;
        }
        if (e2a != last_e2a)
        {
            format_edge_count('2', 'A', e2a, edge_text);
            LCD_ShowString(4, 100, (const unsigned char *)edge_text, YELLOW, BLACK, 32, 0);
            last_e2a = e2a;
        }

        delay_cycles(CPUCLK_FREQ / 20U);
    }
}

static void format_edge_count(char encoder, char phase, uint32_t count, char text[10])
{
    if (count > 99999U) {
        count = 99999U;
    }

    text[0] = encoder;
    text[1] = phase;
    text[2] = ':';
    text[3] = (char)('0' + ((count / 10000U) % 10U));
    text[4] = (char)('0' + ((count / 1000U) % 10U));
    text[5] = (char)('0' + ((count / 100U) % 10U));
    text[6] = (char)('0' + ((count / 10U) % 10U));
    text[7] = (char)('0' + (count % 10U));
    text[8] = '\0';
}

static void format_track_error(int16_t error, char text[7])
{
    uint16_t magnitude;

    if (error > 99) {
        error = 99;
    } else if (error < -99) {
        error = -99;
    }

    text[0] = 'E';
    text[1] = ':';
    text[2] = (error < 0) ? '-' : '+';
    magnitude = (error < 0) ? (uint16_t)(-error) : (uint16_t)error;
    text[3] = (char)('0' + ((magnitude / 10U) % 10U));
    text[4] = (char)('0' + (magnitude % 10U));
    text[5] = '\0';
}

static void track_sensor_test(void)
{
    static const int8_t weights[8] = {-35, -25, -15, -5, 5, 15, 25, 35};
    uint8_t raw;
    uint8_t line_mask;
    uint8_t last_raw = 0xFFU;
    uint8_t index;
    uint8_t active_count;
    int16_t weighted_sum;
    int16_t error;
    char bits[9];
    char error_text[7];

    LCD_BLK_Set();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_ShowString(4, 4, (const unsigned char *)"TRACK POS", WHITE, BLACK, 24, 0);
    LCD_ShowString(4, 42, (const unsigned char *)"12345678", CYAN, BLACK, 32, 0);

    while (1)
    {
        raw = line_sensor_port_read_raw();
        if (raw != last_raw)
        {
            active_count = 0U;
            weighted_sum = 0;
            line_mask = (uint8_t)(~raw);

            for (index = 0U; index < 8U; index++)
            {
                bits[index] = ((raw & (1U << index)) != 0U) ? '1' : '0';
                if ((line_mask & (1U << index)) != 0U)
                {
                    active_count++;
                    weighted_sum += weights[index];
                }
            }
            bits[8] = '\0';
            LCD_ShowString(4, 86, (const unsigned char *)bits, YELLOW, BLACK, 32, 0);

            LCD_Fill(0, 126, 180, 165, BLACK);
            if (active_count == 0U)
            {
                LCD_ShowString(4, 130, (const unsigned char *)"LOST", RED, BLACK, 32, 0);
            }
            else if (active_count >= 6U)
            {
                LCD_ShowString(4, 130, (const unsigned char *)"STOP", GREEN, BLACK, 32, 0);
            }
            else
            {
                error = weighted_sum / (int16_t)active_count;
                format_track_error(error, error_text);
                LCD_ShowString(4, 130, (const unsigned char *)error_text, GREEN, BLACK, 32, 0);
            }

            last_raw = raw;
        }

        delay_cycles(CPUCLK_FREQ / 50U);
    }
}

static void track_motor_test(void)
{
    static const int8_t weights[8] = {-35, -25, -15, -5, 5, 15, 25, 35};
    const int16_t base_duty = 18;
    uint8_t raw;
    uint8_t line_mask;
    uint8_t index;
    uint8_t active_count;
    uint16_t last_raw = 0x100U;
    int16_t weighted_sum;
    int16_t error;
    int16_t correction;
    int16_t left_duty;
    int16_t right_duty;
    char bits[9];
    char error_text[7];

    chassis_actuator_init();
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);

    LCD_BLK_Set();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_ShowString(4, 4, (const unsigned char *)"TRACK MOTOR", WHITE, BLACK, 24, 0);
    LCD_ShowString(4, 42, (const unsigned char *)"12345678", CYAN, BLACK, 32, 0);

    chassis_actuator_enable();

    while (1)
    {
        raw = line_sensor_port_read_raw();
        if ((uint16_t)raw != last_raw)
        {
            active_count = 0U;
            weighted_sum = 0;
            line_mask = (uint8_t)(~raw);

            for (index = 0U; index < 8U; index++)
            {
                bits[index] = ((raw & (1U << index)) != 0U) ? '1' : '0';
                if ((line_mask & (1U << index)) != 0U)
                {
                    active_count++;
                    weighted_sum += weights[index];
                }
            }
            bits[8] = '\0';
            LCD_ShowString(4, 86, (const unsigned char *)bits, YELLOW, BLACK, 32, 0);
            LCD_Fill(0, 126, 180, 165, BLACK);

            if (active_count == 0U)
            {
                chassis_actuator_stop();
                chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
                chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
                LCD_ShowString(4, 130, (const unsigned char *)"LOST", RED, BLACK, 32, 0);
            }
            else if (active_count >= 6U)
            {
                chassis_actuator_stop();
                chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
                chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
                LCD_ShowString(4, 130, (const unsigned char *)"STOP", GREEN, BLACK, 32, 0);
            }
            else
            {
                error = weighted_sum / (int16_t)active_count;
                correction = error / 5;
                left_duty = base_duty + correction;
                right_duty = base_duty - correction;

                chassis_actuator_set_pwm((float)left_duty, (float)right_duty);
                format_track_error(error, error_text);
                LCD_ShowString(4, 130, (const unsigned char *)error_text, GREEN, BLACK, 32, 0);
            }

            last_raw = raw;
        }

        delay_cycles(CPUCLK_FREQ / 100U);
    }
}

static void track_ground_test(void)
{
    static const int8_t weights[8] = {-35, -25, -15, -5, 5, 15, 25, 35};
    const int16_t base_duty = 25;
    uint8_t raw;
    uint8_t line_mask;
    uint8_t index;
    uint8_t active_count;
    int16_t weighted_sum;
    int16_t error;
    int16_t correction;
    int16_t left_duty;
    int16_t right_duty;
    uint32_t run_start_ms;

    chassis_actuator_init();
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
    chassis_actuator_disable();

    LCD_BLK_Set();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_ShowString(4, 4, (const unsigned char *)"GROUND TEST", WHITE, BLACK, 24, 0);
    LCD_ShowString(4, 52, (const unsigned char *)"B21 START", CYAN, BLACK, 32, 0);
    LCD_ShowString(4, 104, (const unsigned char *)"5 SEC", YELLOW, BLACK, 32, 0);

    mspm0_delay_ms(2000U);

    raw = line_sensor_port_read_raw();
    line_mask = (uint8_t)(~raw);
    if ((line_mask == 0U) || (line_mask == 0xFFU))
    {
        LCD_Fill(0, 48, 220, 150, BLACK);
        LCD_ShowString(4, 68, (const unsigned char *)"NO LINE", RED, BLACK, 32, 0);
        while (1) {
            __WFI();
        }
    }

    chassis_actuator_enable();
    run_start_ms = tick_ms;
    LCD_Fill(0, 48, 220, 150, BLACK);
    LCD_ShowString(4, 68, (const unsigned char *)"RUN", GREEN, BLACK, 32, 0);

    while ((uint32_t)(tick_ms - run_start_ms) < 5000U)
    {
        raw = line_sensor_port_read_raw();
        line_mask = (uint8_t)(~raw);
        active_count = 0U;
        weighted_sum = 0;

        for (index = 0U; index < 8U; index++)
        {
            if ((line_mask & (1U << index)) != 0U)
            {
                active_count++;
                weighted_sum += weights[index];
            }
        }

        if ((active_count == 0U) || (active_count >= 6U))
        {
            chassis_actuator_stop();
            chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
            chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
            chassis_actuator_disable();
            LCD_Fill(0, 48, 220, 150, BLACK);
            if (active_count == 0U) {
                LCD_ShowString(4, 68, (const unsigned char *)"LOST STOP", RED, BLACK, 32, 0);
            } else {
                LCD_ShowString(4, 68, (const unsigned char *)"LINE STOP", GREEN, BLACK, 32, 0);
            }
            while (1) {
                __WFI();
            }
        }

        error = weighted_sum / (int16_t)active_count;
        correction = error / 3;
        left_duty = base_duty + correction;
        right_duty = base_duty - correction;
        chassis_actuator_set_pwm((float)left_duty, (float)right_duty);

        delay_cycles(CPUCLK_FREQ / 100U);
    }

    chassis_actuator_stop();
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
    chassis_actuator_disable();
    LCD_Fill(0, 48, 220, 150, BLACK);
    LCD_ShowString(4, 68, (const unsigned char *)"TIME STOP", GREEN, BLACK, 32, 0);

    while (1)
    {
        __WFI();
    }
}

static void format_lap_time(uint32_t elapsed_ms, char text[8])
{
    uint32_t tenths = elapsed_ms / 100U;

    if (tenths > 999U) {
        tenths = 999U;
    }

    text[0] = 'T';
    text[1] = ':';
    text[2] = (char)('0' + ((tenths / 100U) % 10U));
    text[3] = (char)('0' + ((tenths / 10U) % 10U));
    text[4] = '.';
    text[5] = (char)('0' + (tenths % 10U));
    text[6] = '\0';
}

static void competition_runtime_wait_ms(uint32_t duration_ms)
{
    const uint32_t start_ms = tick_ms;

    while ((uint32_t)(tick_ms - start_ms) < duration_ms)
    {
        hball_runtime_target_poll(tick_ms);
        __WFI();
    }
    hball_runtime_target_poll(tick_ms);
}

static void mission_lcd_fill_serviced(
    unsigned int xsta,
    unsigned int ysta,
    unsigned int xend,
    unsigned int yend,
    unsigned int color
)
{
    unsigned int row = ysta;

    while (row < yend)
    {
        unsigned int next_row = row + 8U;

        if (next_row > yend)
        {
            next_row = yend;
        }
        LCD_Fill(xsta, row, xend, next_row, color);
        hball_runtime_target_poll(tick_ms);
        row = next_row;
    }
}

static uint8_t select_car_task(void)
{
    hball_mission_client_t snapshot;
    hball_mission_menu_view_t last_view;
    hball_mission_menu_view_t view;
    hball_mission_menu_event_t mission_event;
    hball_mission_menu_result_t result;
    task_key_event_t key_event;
    bool last_view_valid = false;
    uint32_t last_render_ms = 0U;

    mission_lcd_fill_serviced(0, 0, LCD_W, LCD_H, BLACK);

    while (1)
    {
        hball_runtime_target_poll(tick_ms);
        if (hball_can_mission_get_snapshot(&snapshot)
            && hball_mission_menu_make_view(&snapshot, tick_ms, &view)
            && (!last_view_valid
                || ((uint32_t)(tick_ms - last_render_ms)
                    >= HBALL_MISSION_MENU_RENDER_MIN_MS))
            && (!last_view_valid
                || !hball_mission_menu_view_equal(&last_view, &view)))
        {
            render_mission_menu(&view);
            last_view = view;
            last_view_valid = true;
            last_render_ms = tick_ms;
        }
        key_event = get_task_key_event();
        if (hball_can_mission_get_snapshot(&snapshot)
            && (snapshot.selected_mission
                >= HBALL_MISSION_Q3_BALL_SEQUENCE)
            && (get_q3_level_key_event() != 0U))
        {
            if (hball_can_mission_request_level(tick_ms))
            {
                beep();
            }
            continue;
        }
        if (key_event == TASK_KEY_EVENT_NONE)
        {
            competition_runtime_wait_ms(5U);
            continue;
        }
        if (key_event == TASK_KEY_EVENT_SELECT)
        {
            mission_event = HBALL_MISSION_MENU_SELECT;
        }
        else if (key_event == TASK_KEY_EVENT_EXECUTE)
        {
            mission_event = HBALL_MISSION_MENU_EXECUTE;
        }
        else
        {
            continue;
        }
        result = hball_can_mission_menu_handle(mission_event, tick_ms);
        if (result == HBALL_MISSION_MENU_SELECTED)
        {
            beep();
        }
        else if (result == HBALL_MISSION_MENU_LOCAL_START_ACCEPTED)
        {
            beep();
            delay_cycles(CPUCLK_FREQ / 20U);
            beep();
            return HBALL_MISSION_Q2_FAST_LAP;
        }
        else if (result == HBALL_MISSION_MENU_START_ACCEPTED)
        {
            hball_mission_policy_t policy;
            uint8_t selected_mission;

            beep();
            competition_runtime_wait_ms(50U);
            beep();
            if (!hball_can_mission_get_snapshot(&snapshot))
            {
                continue;
            }
            selected_mission = snapshot.selected_mission;
            if (!hball_mission_policy_get(selected_mission, &policy))
            {
                continue;
            }
            while (1)
            {
                if (hball_can_mission_get_snapshot(&snapshot)
                    && snapshot.status_valid
                    && (snapshot.latest_status.global_state
                        == HBALL_MISSION_STATE_RUNNING)
                    && policy.chassis_allowed)
                {
                    return selected_mission;
                }
                if (hball_can_mission_get_snapshot(&snapshot)
                    && snapshot.status_valid
                    && ((snapshot.latest_status.global_state
                         == HBALL_MISSION_STATE_COMPLETED)
                        || (snapshot.latest_status.global_state
                            >= HBALL_MISSION_STATE_CONTROLLED_ABORT)))
                {
                    hball_can_mission_chassis_finish(
                        HBALL_MISSION_CHASSIS_EVENT_STOPPED, tick_ms
                    );
                    break;
                }
                competition_runtime_wait_ms(5U);
            }
        }
        else if ((result == HBALL_MISSION_MENU_START_BLOCKED)
                 || (result == HBALL_MISSION_MENU_LOCKED))
        {
            beep();
        }
    }
}

static void render_mission_menu(
    const hball_mission_menu_view_t *view
)
{
    char ready_text[5];
    int32_t motor_mrad;
    int32_t target_cm;

    if (view == NULL)
    {
        return;
    }
    format_hex16(view->ready_mask, ready_text);

    mission_lcd_fill_serviced(0, 0, LCD_W, 39, BLACK);
    LCD_ShowString(
        4, 4,
        (const unsigned char *)view->mission_label,
        GREEN, BLACK, 32, 0
    );
    mission_lcd_fill_serviced(0, 40, LCD_W, 71, BLACK);
    LCD_ShowString(
        4, 44, (const unsigned char *)view->state_label,
        ((view->local_execution || view->status_fresh)
         && (view->global_state == HBALL_MISSION_STATE_READY))
            ? GREEN : CYAN,
        BLACK, 24, 0
    );
    mission_lcd_fill_serviced(0, 72, LCD_W, 103, BLACK);
    LCD_ShowString(4, 76, (const unsigned char *)"E:", WHITE, BLACK, 24, 0);
    LCD_ShowIntNum(36, 76, view->epoch, 4, WHITE, BLACK, 24);
    LCD_ShowString(116, 76, (const unsigned char *)"R:", WHITE, BLACK, 24, 0);
    LCD_ShowString(
        148, 76, (const unsigned char *)ready_text,
        WHITE, BLACK, 24, 0
    );
    mission_lcd_fill_serviced(0, 104, LCD_W, 135, BLACK);
    LCD_ShowString(4, 108, (const unsigned char *)"MISS:", YELLOW, BLACK, 24, 0);
    LCD_ShowString(
        76, 108, (const unsigned char *)view->missing_label,
        (view->missing_label[0] == 'A') ? GREEN : YELLOW,
        BLACK, 24, 0
    );
    mission_lcd_fill_serviced(0, 136, LCD_W, 203, BLACK);
    if (view->setup_valid
        && ((view->setup_flags & HBALL_MISSION_SETUP_MOTOR_VALID) != 0U))
    {
        motor_mrad = view->motor_angle_mrad;
        LCD_ShowString(4, 140, (const unsigned char *)"M:", WHITE, BLACK, 24, 0);
        LCD_ShowString(
            28, 140,
            (const unsigned char *)((motor_mrad < 0) ? "-" : "+"),
            CYAN, BLACK, 24, 0
        );
        if (motor_mrad < 0)
        {
            motor_mrad = -motor_mrad;
        }
        LCD_ShowIntNum(40, 140, (uint32_t)(motor_mrad / 1000), 1, CYAN, BLACK, 24);
        LCD_ShowString(56, 140, (const unsigned char *)".", CYAN, BLACK, 24, 0);
        LCD_ShowIntNum(68, 140, (uint32_t)(motor_mrad % 1000), 3, CYAN, BLACK, 24);
        LCD_ShowString(116, 140, (const unsigned char *)"RAD", WHITE, BLACK, 24, 0);
    }
    else
    {
        LCD_ShowString(4, 140, (const unsigned char *)"M: ---.--- RAD", YELLOW, BLACK, 24, 0);
    }
    if (view->mission_id == HBALL_MISSION_Q6_HOLD_POSITION_LAP)
    {
        target_cm = view->target_position_mm / 10;
        LCD_ShowString(4, 172, (const unsigned char *)"Q6 T:", WHITE, BLACK, 24, 0);
        LCD_ShowString(
            64, 172,
            (const unsigned char *)((target_cm < 0) ? "-" : "+"),
            GREEN, BLACK, 24, 0
        );
        if (target_cm < 0)
        {
            target_cm = -target_cm;
        }
        LCD_ShowIntNum(76, 172, (uint32_t)target_cm, 2, GREEN, BLACK, 24);
        LCD_ShowString(108, 172, (const unsigned char *)"CM PB21+1", WHITE, BLACK, 24, 0);
    }
    else if (view->start_requested)
    {
        LCD_ShowString(4, 172, (const unsigned char *)"START SENT", MAGENTA, BLACK, 24, 0);
    }
    else if ((view->mission_id >= HBALL_MISSION_Q3_BALL_SEQUENCE)
             && (view->mission_id <= HBALL_MISSION_Q5_CENTER_LAP))
    {
        LCD_ShowString(4, 172, (const unsigned char *)"PB21 LEVEL SW1 GO", WHITE, BLACK, 24, 0);
    }
    else
    {
        LCD_ShowString(4, 172, (const unsigned char *)"SW3 SELECT SW1 GO", WHITE, BLACK, 24, 0);
    }
}

static void format_hex16(uint16_t value, char text[5])
{
    static const char digits[] = "0123456789ABCDEF";
    uint8_t index;

    for (index = 0U; index < 4U; ++index)
    {
        const uint8_t shift = (uint8_t)((3U - index) * 4U);
        text[index] = digits[(value >> shift) & 0x0FU];
    }
    text[4] = '\0';
}

static void speed_calibration_test(void)
{
    const uint32_t test_time_ms = 5000U;
    const uint32_t sample_time_ms = 100U;
    uint32_t test_start_ms;
    uint32_t last_sample_ms;
    int32_t previous_left;
    int32_t previous_right;
    int32_t current_left;
    int32_t current_right;
    int32_t delta_left;
    int32_t delta_right;
    int32_t average_left;
    int32_t average_right;
    char count_text[10];

    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    encoder_e1a_edges = 0U;
    encoder_e2a_edges = 0U;

    DL_GPIO_clearInterruptStatus(ENCODERA_PORT, ENCODERA_E1A_PIN);
    DL_GPIO_clearInterruptStatus(ENCODERB_PORT, ENCODERB_E2A_PIN);
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);

    chassis_actuator_init();
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
    chassis_actuator_disable();

    LCD_BLK_Set();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_ShowString(4, 4, (const unsigned char *)"SPEED CAL 30", WHITE, BLACK, 24, 0);
    LCD_ShowString(4, 42, (const unsigned char *)"AUTO 2 SEC", CYAN, BLACK, 32, 0);
    telemetry_send_string("SPEED_CAL_READY,AUTO_START_2S\r\n");
    mspm0_delay_ms(2000U);

    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    previous_left = 0;
    previous_right = 0;
    chassis_actuator_enable();
    chassis_actuator_set_pwm(30.0f, 30.0f);
    test_start_ms = tick_ms;
    last_sample_ms = test_start_ms;
    LCD_Fill(0, 38, 260, 165, BLACK);
    LCD_ShowString(4, 42, (const unsigned char *)"RUN 5 SEC", GREEN, BLACK, 24, 0);

    while ((uint32_t)(tick_ms - test_start_ms) < test_time_ms)
    {
        if ((uint32_t)(tick_ms - last_sample_ms) >= sample_time_ms)
        {
            __disable_irq();
            current_left = Get_Encoder_countA;
            current_right = Get_Encoder_countB;
            __enable_irq();

            delta_left = current_left - previous_left;
            delta_right = current_right - previous_right;
            previous_left = current_left;
            previous_right = current_right;

            format_encoder_count('L', delta_left, count_text);
            LCD_ShowString(4, 76, (const unsigned char *)count_text, CYAN, BLACK, 32, 0);
            format_encoder_count('R', delta_right, count_text);
            LCD_ShowString(4, 116, (const unsigned char *)count_text, YELLOW, BLACK, 32, 0);
            snprintf((char *)uart_send, sizeof(uart_send),
                     "S,L=%ld,R=%ld,PL=30,PR=30\r\n",
                     (long)delta_left, (long)delta_right);
            telemetry_send_string((char *)uart_send);
            last_sample_ms = tick_ms;
        }

        delay_cycles(CPUCLK_FREQ / 1000U);
    }

    chassis_actuator_stop();
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
    chassis_actuator_disable();

    __disable_irq();
    current_left = Get_Encoder_countA;
    current_right = Get_Encoder_countB;
    __enable_irq();
    average_left = current_left / (int32_t)(test_time_ms / sample_time_ms);
    average_right = current_right / (int32_t)(test_time_ms / sample_time_ms);

    LCD_Fill(0, 36, LCD_W, LCD_H, BLACK);
    LCD_ShowString(4, 40, (const unsigned char *)"AVG/100MS", WHITE, BLACK, 24, 0);
    format_encoder_count('L', average_left, count_text);
    LCD_ShowString(4, 76, (const unsigned char *)count_text, CYAN, BLACK, 32, 0);
    format_encoder_count('R', average_right, count_text);
    LCD_ShowString(4, 116, (const unsigned char *)count_text, YELLOW, BLACK, 32, 0);
    snprintf((char *)uart_send, sizeof(uart_send),
             "AVG,L=%ld,R=%ld,TL=%ld,TR=%ld\r\n",
             (long)average_left, (long)average_right,
             (long)current_left, (long)current_right);
    telemetry_send_string((char *)uart_send);

    while (1)
    {
        __WFI();
    }
}

static void motor_encoder_map_test(void)
{
    static int32_t test_counts_a[2];
    static int32_t test_counts_b[2];
    const uint32_t measure_time_ms = 1500U;
    uint32_t stage_start_ms;
    uint8_t i;

    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    encoder_e1a_edges = 0U;
    encoder_e2a_edges = 0U;
    DL_GPIO_clearInterruptStatus(ENCODERA_PORT, ENCODERA_E1A_PIN);
    DL_GPIO_clearInterruptStatus(ENCODERB_PORT, ENCODERB_E2A_PIN);
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);

    chassis_actuator_init();
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
    chassis_actuator_disable();

    LCD_BLK_Set();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_ShowString(4, 4, (const unsigned char *)"MOTOR MAP", WHITE, BLACK, 32, 0);
    LCD_ShowString(4, 58, (const unsigned char *)"AUTO 2 SEC", CYAN, BLACK, 24, 0);
    telemetry_send_string("MAP_READY,AUTO_START_2S\r\n");
    mspm0_delay_ms(2000U);

    for (i = 0U; i < 2U; i++)
    {
        chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
        chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
        chassis_actuator_disable();
        mspm0_delay_ms(600U);

        __disable_irq();
        Get_Encoder_countA = 0;
        Get_Encoder_countB = 0;
        __enable_irq();

        chassis_actuator_enable();
        if (i == 0U) {
            chassis_actuator_set_pwm(25.0f, 0.0f);
        } else {
            chassis_actuator_set_pwm(0.0f, 25.0f);
        }

        stage_start_ms = tick_ms;
        while ((uint32_t)(tick_ms - stage_start_ms) < measure_time_ms)
        {
            delay_cycles(CPUCLK_FREQ / 2000U);
        }

        __disable_irq();
        test_counts_a[i] = Get_Encoder_countA;
        test_counts_b[i] = Get_Encoder_countB;
        __enable_irq();
    }

    chassis_actuator_stop();
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
    chassis_actuator_disable();
    LCD_Fill(0, 50, 250, 120, BLACK);
    LCD_ShowString(4, 58, (const unsigned char *)"MAP DONE", GREEN, BLACK, 32, 0);

    telemetry_send_string("MAP_LOG_BEGIN\r\n");
    snprintf((char *)uart_send, sizeof(uart_send),
             "CMD_LEFT25,A=%ld,B=%ld\r\n",
             (long)test_counts_a[0], (long)test_counts_b[0]);
    telemetry_send_string((char *)uart_send);
    snprintf((char *)uart_send, sizeof(uart_send),
             "CMD_RIGHT25,A=%ld,B=%ld\r\n",
             (long)test_counts_a[1], (long)test_counts_b[1]);
    telemetry_send_string((char *)uart_send);
    telemetry_send_string("MAP_LOG_END\r\n");

    while (1)
    {
        __WFI();
    }
}

static void pwm_sweep_test(void)
{
    enum { SWEEP_POINTS = 9 };
    static const uint8_t duty_points[SWEEP_POINTS] = {
        12U, 15U, 18U, 20U, 22U, 25U, 30U, 35U, 40U
    };
    static int32_t left_counts[SWEEP_POINTS];
    static int32_t right_counts[SWEEP_POINTS];
    const uint32_t settle_time_ms = 400U;
    const uint32_t measure_time_ms = 1500U;
    uint32_t stage_start_ms;
    uint8_t i;

    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    encoder_e1a_edges = 0U;
    encoder_e2a_edges = 0U;
    DL_GPIO_clearInterruptStatus(ENCODERA_PORT, ENCODERA_E1A_PIN);
    DL_GPIO_clearInterruptStatus(ENCODERB_PORT, ENCODERB_E2A_PIN);
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);

    chassis_actuator_init();
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
    chassis_actuator_disable();

    LCD_BLK_Set();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_ShowString(4, 4, (const unsigned char *)"PWM SWEEP", WHITE, BLACK, 32, 0);
    LCD_ShowString(4, 58, (const unsigned char *)"SW1 OR G", CYAN, BLACK, 24, 0);
    telemetry_send_string("SWEEP_READY,SEND_G_OR_PRESS_SW1\r\n");
    while (!speed_calibration_start_requested())
    {
        hball_runtime_target_poll(tick_ms);
        __WFI();
    }

    LCD_Fill(0, 50, 240, 110, BLACK);
    LCD_ShowString(4, 58, (const unsigned char *)"RUNNING", GREEN, BLACK, 32, 0);

    for (i = 0U; i < SWEEP_POINTS; i++)
    {
        chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
        chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
        chassis_actuator_disable();
        mspm0_delay_ms(settle_time_ms);

        __disable_irq();
        Get_Encoder_countA = 0;
        Get_Encoder_countB = 0;
        __enable_irq();

        chassis_actuator_enable();
        chassis_actuator_set_pwm((float)duty_points[i], (float)duty_points[i]);
        stage_start_ms = tick_ms;
        while ((uint32_t)(tick_ms - stage_start_ms) < measure_time_ms)
        {
            delay_cycles(CPUCLK_FREQ / 2000U);
        }

        __disable_irq();
        /* Output 1/left is paired with countB; output 2/right with countA. */
        left_counts[i] = Get_Encoder_countB;
        right_counts[i] = Get_Encoder_countA;
        __enable_irq();
    }

    chassis_actuator_stop();
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
    chassis_actuator_disable();
    LCD_Fill(0, 50, 250, 120, BLACK);
    LCD_ShowString(4, 58, (const unsigned char *)"SWEEP DONE", GREEN, BLACK, 32, 0);

    telemetry_send_string("SWEEP_LOG_BEGIN\r\n");
    for (i = 0U; i < SWEEP_POINTS; i++)
    {
        snprintf((char *)uart_send, sizeof(uart_send),
                 "D=%u,L=%ld,R=%ld,MS=%lu\r\n",
                 (unsigned)duty_points[i],
                 (long)left_counts[i],
                 (long)right_counts[i],
                 (unsigned long)measure_time_ms);
        telemetry_send_string((char *)uart_send);
    }
    telemetry_send_string("SWEEP_LOG_END\r\n");

    while (1)
    {
        __WFI();
    }
}

static void speed_pi_test(void)
{
    enum { PI_LOG_SAMPLES = 40 };
    static uint16_t log_time_ms[PI_LOG_SAMPLES];
    static uint16_t log_scale_permille[PI_LOG_SAMPLES];
    static int16_t log_target[PI_LOG_SAMPLES];
    static int16_t log_left_speed[PI_LOG_SAMPLES];
    static int16_t log_right_speed[PI_LOG_SAMPLES];
    static int16_t log_left_duty[PI_LOG_SAMPLES];
    static int16_t log_right_duty[PI_LOG_SAMPLES];
    const uint32_t test_time_ms = 3300U;
    const uint32_t brake_start_ms = 2500U;
    const int16_t cruise_target = 50;
    chassis_motion_profile_t speed_profile;
    wheel_control_t wheel_control;
    motion_intent_t intent;
    wheel_control_output_t output;
    uint32_t test_start_ms;
    uint32_t elapsed_ms;
    uint16_t log_index = 0U;
    int32_t current_left_count;
    int32_t current_right_count;
    float speed_scale = 0.0F;
    bool braking = false;

    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    encoder_e1a_edges = 0U;
    encoder_e2a_edges = 0U;
    DL_GPIO_clearInterruptStatus(ENCODERA_PORT, ENCODERA_E1A_PIN);
    DL_GPIO_clearInterruptStatus(ENCODERB_PORT, ENCODERB_E2A_PIN);
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);

    chassis_actuator_init();
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
    chassis_actuator_disable();

    LCD_BLK_Set();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_ShowString(4, 4, (const unsigned char *)"S CURVE", WHITE, BLACK, 32, 0);
    LCD_ShowString(4, 58, (const unsigned char *)"SW1 OR G", CYAN, BLACK, 24, 0);
    telemetry_send_string("SCURVE_READY,SEND_G_OR_PRESS_SW1\r\n");
    while (!speed_calibration_start_requested())
    {
        hball_runtime_target_poll(tick_ms);
        __WFI();
    }

    LCD_Fill(0, 50, 240, 110, BLACK);
    LCD_ShowString(4, 58, (const unsigned char *)"RUN 3.3 SEC", GREEN, BLACK, 24, 0);
    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    chassis_actuator_start_synchronized(0.0F, 0.0F);
    test_start_ms = tick_ms;
    wheel_control_init(&wheel_control, test_start_ms, 0, 0);
    chassis_motion_profile_start(
        &speed_profile, 0.0F, 1.0F, test_start_ms, HBALL_Q4_SOFT_START_MS
    );
    memset(&intent, 0, sizeof(intent));
    intent.valid = true;
    intent.duty_slew_step = HBALL_Q4_DUTY_SLEW_STEP;
    while ((uint32_t)(tick_ms - test_start_ms) < test_time_ms)
    {
        elapsed_ms = (uint32_t)(tick_ms - test_start_ms);
        if (!braking && (elapsed_ms >= brake_start_ms))
        {
            braking = true;
            chassis_motion_profile_start(
                &speed_profile,
                speed_scale,
                0.0F,
                tick_ms,
                HBALL_Q4_SOFT_STOP_MS
            );
        }
        speed_scale = chassis_motion_profile_sample(&speed_profile, tick_ms);
        intent.timestamp_ms = tick_ms;
        intent.requested_speed_left = chassis_motion_profile_scale_i16(
            cruise_target, speed_scale
        );
        intent.requested_speed_right = intent.requested_speed_left;

        if (wheel_control_due(&wheel_control, tick_ms))
        {
            __disable_irq();
            current_left_count = Get_Encoder_countB;
            current_right_count = Get_Encoder_countA;
            __enable_irq();
            if (wheel_control_step(
                    &wheel_control,
                    tick_ms,
                    current_left_count,
                    current_right_count,
                    &intent,
                    &output))
            {
                chassis_actuator_set_pwm(
                    (float)output.duty_left, (float)output.duty_right
                );
                if (log_index < PI_LOG_SAMPLES)
                {
                    log_time_ms[log_index] = (uint16_t)elapsed_ms;
                    log_scale_permille[log_index] =
                        (uint16_t)(speed_scale * 1000.0F);
                    log_target[log_index] = intent.requested_speed_left;
                    log_left_speed[log_index] =
                        (int16_t)output.measured_left_speed;
                    log_right_speed[log_index] =
                        (int16_t)output.measured_right_speed;
                    log_left_duty[log_index] = output.duty_left;
                    log_right_duty[log_index] = output.duty_right;
                    log_index++;
                }
            }
        }
        competition_runtime_wait_ms(10U);
    }

    chassis_actuator_stop();
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
    chassis_actuator_disable();
    LCD_Fill(0, 50, 250, 120, BLACK);
    LCD_ShowString(4, 58, (const unsigned char *)"PI DONE", GREEN, BLACK, 32, 0);

    telemetry_send_string("SCURVE_LOG_BEGIN,t,scale,target,left,dl,right,dr\r\n");
    for (uint16_t i = 0U; i < log_index; i++)
    {
        snprintf((char *)uart_send, sizeof(uart_send),
                 "C,%u,%u,%d,%d,%d,%d,%d\r\n",
                 (unsigned)log_time_ms[i],
                 (unsigned)log_scale_permille[i],
                 (int)log_target[i],
                 (int)log_left_speed[i],
                 (int)log_left_duty[i],
                 (int)log_right_speed[i],
                 (int)log_right_duty[i]);
        telemetry_send_string((char *)uart_send);
    }
    telemetry_send_string("SCURVE_LOG_END\r\n");

    while (1)
    {
        __WFI();
    }
}

static void lap_test(void)
{
    while (1)
    {
        lap_test_once();
    }
}

static void lap_test_once(void)
{
    enum { LAP_LOG_SAMPLES = 350 };
    static uint16_t log_time_ms[LAP_LOG_SAMPLES];
    static uint8_t log_line_mask[LAP_LOG_SAMPLES];
    static int8_t log_error[LAP_LOG_SAMPLES];
    static uint8_t log_target_left[LAP_LOG_SAMPLES];
    static uint8_t log_target_right[LAP_LOG_SAMPLES];
    static int16_t log_actual_left[LAP_LOG_SAMPLES];
    static int16_t log_actual_right[LAP_LOG_SAMPLES];
    static uint8_t log_duty_left[LAP_LOG_SAMPLES];
    static uint8_t log_duty_right[LAP_LOG_SAMPLES];
    hball_mission_client_t mission_snapshot;
    hball_mission_policy_t mission_policy;
    hball_mission_run_decision_t run_decision;
    task_key_event_t run_key_event;
    static line_follower_t line_follower;
    static route_marker_detector_t route_marker_detector;
    static wheel_control_t wheel_control;
    chassis_motion_profile_t q4_speed_profile;
    line_follower_output_t follower_output;
    line_follower_profile_t follower_profile;
    motion_intent_t wheel_intent;
    route_marker_detector_config_t marker_config;
    route_marker_detector_output_t marker_output;
    wheel_control_output_t wheel_output;
    uint8_t mission_id;
    uint8_t selected_task;
    uint8_t finish_event_flags = HBALL_MISSION_CHASSIS_EVENT_STOPPED;
    bool local_marker_stop_enabled;
    bool q4_braking = false;
    bool q56_braking = false;
    bool q56_stop_profile_complete = false;
    bool q56_stop_settle_active = false;
    bool q56_stop_complete = false;
    bool wheel_sample_valid = false;
    uint16_t mission_epoch;
    uint32_t run_timeout_ms;
    line_snapshot_t line_sample;
    uint8_t line_mask;
    uint16_t log_index = 0U;
    int16_t error = 0;
    int16_t commanded_duty_left = 15;
    int16_t commanded_duty_right = 15;
    int32_t current_left_count;
    int32_t current_right_count;
    int32_t left_speed;
    int32_t right_speed;
    float q4_speed_scale = 1.0F;
    float q56_speed_scale = 1.0F;
    uint32_t run_start_ms;
    uint32_t elapsed_ms;
    uint32_t q56_stop_profile_complete_ms = 0U;
    uint32_t q56_stop_settle_start_ms = 0U;
    uint32_t last_wheel_sample_ms = 0U;
    uint32_t finish_elapsed_ms = 0U;
    char time_text[8];

    chassis_actuator_init();
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
    chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
    chassis_actuator_disable();

    hball_runtime_services_enter_menu();
    LCD_BLK_Set();
    selected_task = select_car_task();
    mission_id = selected_task;
    if (!hball_mission_policy_get(mission_id, &mission_policy)
        || !hball_can_mission_get_snapshot(&mission_snapshot)
        || (mission_snapshot.selected_mission != mission_id))
    {
        hball_runtime_services_apply_policy(NULL);
        return;
    }
    mission_epoch = mission_snapshot.candidate_epoch;
    hball_runtime_services_apply_policy(&mission_policy);
    marker_config.marker_active_threshold = 3U;
    marker_config.marker_adjacent_width = 0U;
    marker_config.start_clear_max_active = 4U;
    marker_config.start_clear_confirm_ms = 120U;
    marker_config.marker_min_elapsed_ms = 10000U;
    marker_config.marker_confirm_ms = 0U;
    if (selected_task == HBALL_MISSION_Q2_FAST_LAP)
    {
        selected_task = CAR_TASK_LAP_STOP;
        follower_profile = LINE_FOLLOWER_PROFILE_Q2_FAST_LAP;
        local_marker_stop_enabled = true;
        marker_config.marker_active_threshold = 3U;
        marker_config.marker_adjacent_width = 3U;
        marker_config.marker_min_elapsed_ms = 18000U;
        marker_config.marker_confirm_ms = 20U;
        run_timeout_ms = 0U;
    }
    else if (selected_task == HBALL_MISSION_Q4_A_TO_B)
    {
        selected_task = CAR_TASK_TIMED_RUN;
        follower_profile = LINE_FOLLOWER_PROFILE_Q4_TIMED_RUN;
        local_marker_stop_enabled = false;
        run_timeout_ms = 7800U;
    }
    else
    {
        selected_task = CAR_TASK_STABLE_LAP;
        follower_profile = LINE_FOLLOWER_PROFILE_STABLE_LAP;
        local_marker_stop_enabled = true;
        marker_config.marker_active_threshold = 4U;
        marker_config.marker_adjacent_width = 4U;
        marker_config.marker_min_elapsed_ms = 23000U;
        marker_config.marker_confirm_ms = 20U;
        run_timeout_ms = 0U;
    }
    line_sample = line_snapshot_decode(line_sensor_port_read_raw(), tick_ms);
    line_mask = line_sample.line_mask;
    if (line_mask == 0U)
    {
        mission_lcd_fill_serviced(0, 48, 240, 150, BLACK);
        LCD_ShowString(4, 68, (const unsigned char *)"NO LINE", RED, BLACK, 32, 0);
        (void)hball_can_mission_request_abort(tick_ms);
        hball_can_mission_chassis_finish(
            HBALL_MISSION_CHASSIS_EVENT_STOPPED
                | HBALL_MISSION_CHASSIS_EVENT_LINE_LOST
                | HBALL_MISSION_CHASSIS_EVENT_LOCAL_FAULT,
            tick_ms
        );
        competition_runtime_wait_ms(500U);
        return;
    }

    /*
     * Output channel 1 drives the wheel measured by E1A/countB.
     * Output channel 2 drives the wheel measured by E2A/countA.
     */
    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    encoder_e1a_edges = 0U;
    encoder_e2a_edges = 0U;
    DL_GPIO_clearInterruptStatus(ENCODERA_PORT, ENCODERA_E1A_PIN);
    DL_GPIO_clearInterruptStatus(ENCODERB_PORT, ENCODERB_E2A_PIN);
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);

    (void)hball_can_mission_get_snapshot(&mission_snapshot);
    run_decision = hball_mission_run_guard_evaluate(
        &mission_policy,
        mission_id,
        mission_epoch,
        &mission_snapshot,
        tick_ms
    );
    if (run_decision != HBALL_MISSION_RUN_CONTINUE)
    {
        LCD_Fill(0, 48, 280, 100, BLACK);
        if (run_decision == HBALL_MISSION_RUN_STOP_REMOTE_COMPLETE) {
            LCD_ShowString(4, 58, (const unsigned char *)"M33 DONE", GREEN, BLACK, 32, 0);
        } else if (run_decision == HBALL_MISSION_RUN_STOP_REMOTE_ABORT) {
            LCD_ShowString(4, 58, (const unsigned char *)"M33 ABORT", RED, BLACK, 32, 0);
        } else if (run_decision
                   == HBALL_MISSION_RUN_STOP_REMOTE_UNAVAILABLE) {
            LCD_ShowString(4, 58, (const unsigned char *)"M33 LOST", RED, BLACK, 32, 0);
        }
        hball_can_mission_chassis_finish(
            HBALL_MISSION_CHASSIS_EVENT_STOPPED, tick_ms
        );
        return;
    }

    mission_lcd_fill_serviced(0, 0, LCD_W, LCD_H, BLACK);
    if (selected_task == CAR_TASK_LAP_STOP) {
        LCD_ShowString(4, 4, (const unsigned char *)"TASK1 RUN", GREEN, BLACK, 32, 0);
    } else if (selected_task == CAR_TASK_TIMED_RUN) {
        LCD_ShowString(4, 4, (const unsigned char *)"TASK2 RUN", CYAN, BLACK, 32, 0);
    } else {
        LCD_ShowString(4, 4, (const unsigned char *)"TASK3 RUN", MAGENTA, BLACK, 32, 0);
    }
    LCD_ShowString(4, 106, (const unsigned char *)"T:00.0", YELLOW, BLACK, 32, 0);

    if (selected_task == CAR_TASK_TIMED_RUN)
    {
        commanded_duty_left = 0;
        commanded_duty_right = 0;
    }
    chassis_actuator_start_synchronized((float)commanded_duty_left, (float)commanded_duty_right);
    run_start_ms = tick_ms;
    chassis_motion_profile_start(
        &q4_speed_profile,
        selected_task == CAR_TASK_TIMED_RUN ? 0.0F : 1.0F,
        1.0F,
        run_start_ms,
        selected_task == CAR_TASK_TIMED_RUN ? HBALL_Q4_SOFT_START_MS : 0U
    );
    line_follower_init(&line_follower, follower_profile, run_start_ms);
    (void)route_marker_detector_init(
        &route_marker_detector, &marker_config, run_start_ms);
    wheel_control_init(
        &wheel_control,
        run_start_ms,
        commanded_duty_left,
        commanded_duty_right
    );
    hball_can_mission_chassis_start(run_start_ms);

    while (1)
    {
        elapsed_ms = (uint32_t)(tick_ms - run_start_ms);
        run_key_event = get_task_key_event();
        if (run_key_event == TASK_KEY_EVENT_EXECUTE)
        {
            (void)hball_can_mission_request_abort(tick_ms);
            finish_elapsed_ms = elapsed_ms;
            finish_event_flags =
                HBALL_MISSION_CHASSIS_EVENT_STOPPED
                | HBALL_MISSION_CHASSIS_EVENT_LOCAL_FAULT;
            chassis_actuator_stop();
            chassis_actuator_set_wheel_speed(
                0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
            chassis_actuator_set_wheel_speed(
                0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
            chassis_actuator_disable();
            LCD_Fill(0, 48, 280, 100, BLACK);
            LCD_ShowString(
                4, 58, (const unsigned char *)"SW1 CANCEL",
                RED, BLACK, 32, 0);
            break;
        }
        (void)hball_can_mission_get_snapshot(&mission_snapshot);
        run_decision = hball_mission_run_guard_evaluate(
            &mission_policy,
            mission_id,
            mission_epoch,
            &mission_snapshot,
            tick_ms
        );
        if (run_decision != HBALL_MISSION_RUN_CONTINUE)
        {
            finish_elapsed_ms = elapsed_ms;
            finish_event_flags = HBALL_MISSION_CHASSIS_EVENT_STOPPED;
            chassis_actuator_stop();
            chassis_actuator_set_wheel_speed(
                0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
            chassis_actuator_set_wheel_speed(
                0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
            chassis_actuator_disable();
            LCD_Fill(0, 48, 280, 100, BLACK);
            if (run_decision == HBALL_MISSION_RUN_STOP_REMOTE_COMPLETE) {
                LCD_ShowString(4, 58, (const unsigned char *)"M33 DONE", GREEN, BLACK, 32, 0);
            } else if (run_decision == HBALL_MISSION_RUN_STOP_REMOTE_ABORT) {
                LCD_ShowString(4, 58, (const unsigned char *)"M33 ABORT", RED, BLACK, 32, 0);
            } else if (run_decision
                       == HBALL_MISSION_RUN_STOP_REMOTE_UNAVAILABLE) {
                LCD_ShowString(4, 58, (const unsigned char *)"M33 LOST", RED, BLACK, 32, 0);
            }
            break;
        }
        if ((selected_task == CAR_TASK_TIMED_RUN)
            && !q4_braking
            && (elapsed_ms >= (run_timeout_ms - HBALL_Q4_SOFT_STOP_MS)))
        {
            q4_braking = true;
            chassis_motion_profile_start(
                &q4_speed_profile,
                q4_speed_scale,
                0.0F,
                tick_ms,
                HBALL_Q4_SOFT_STOP_MS
            );
        }
        if ((run_timeout_ms != 0U) && (elapsed_ms >= run_timeout_ms))
        {
            finish_elapsed_ms = run_timeout_ms;
            finish_event_flags = HBALL_MISSION_CHASSIS_EVENT_DETECTED_B;
            chassis_actuator_stop();
            chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
            chassis_actuator_set_wheel_speed(0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
            chassis_actuator_disable();
            LCD_Fill(0, 48, 280, 100, BLACK);
            if (selected_task == CAR_TASK_TIMED_RUN) {
                LCD_ShowString(4, 58, (const unsigned char *)"TASK2 DONE", GREEN, BLACK, 32, 0);
            } else {
                LCD_ShowString(4, 58, (const unsigned char *)"TASK3 DONE", GREEN, BLACK, 32, 0);
            }
            break;
        }

        line_sample = line_snapshot_decode(line_sensor_port_read_raw(), tick_ms);
        line_mask = line_sample.line_mask;
        (void)route_marker_detector_step(
            &route_marker_detector, &line_sample, &marker_output);

        if (marker_output.start_cleared_event)
        {
            hball_can_mission_chassis_latch_events(
                HBALL_MISSION_CHASSIS_EVENT_LEFT_A);
        }
        if (marker_output.marker_confirmed_event)
        {
            hball_can_mission_chassis_latch_events(
                HBALL_MISSION_CHASSIS_EVENT_REACQUIRED_A);
        }

        if (local_marker_stop_enabled && marker_output.marker_confirmed_event)
        {
            if (selected_task == CAR_TASK_STABLE_LAP)
            {
                q56_braking = true;
                chassis_motion_profile_start(
                    &q4_speed_profile,
                    q56_speed_scale,
                    0.0F,
                    tick_ms,
                    HBALL_Q56_SOFT_STOP_MS
                );
            }
            else
            {
                finish_elapsed_ms = elapsed_ms;
                finish_event_flags =
                    HBALL_MISSION_CHASSIS_EVENT_REACQUIRED_A;
                chassis_actuator_stop();
                chassis_actuator_set_wheel_speed(
                    0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
                chassis_actuator_set_wheel_speed(
                    0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
                chassis_actuator_disable();
                LCD_Fill(0, 48, 280, 100, BLACK);
                LCD_ShowString(
                    4, 58, (const unsigned char *)"LAP STOP",
                    GREEN, BLACK, 32, 0);
                break;
            }
        }

        (void)line_follower_step(
            &line_follower,
            &line_sample,
            marker_output.force_straight,
            &follower_output
        );
        wheel_intent = follower_output.intent;
        if (selected_task == CAR_TASK_TIMED_RUN)
        {
            q4_speed_scale = chassis_motion_profile_sample(
                &q4_speed_profile, tick_ms
            );
            wheel_intent.requested_speed_left =
                chassis_motion_profile_scale_i16(
                    wheel_intent.requested_speed_left, q4_speed_scale
                );
            wheel_intent.requested_speed_right =
                chassis_motion_profile_scale_i16(
                    wheel_intent.requested_speed_right, q4_speed_scale
                );
            wheel_intent.duty_slew_step = HBALL_Q4_DUTY_SLEW_STEP;
        }
        else if (q56_braking)
        {
            q56_speed_scale = chassis_motion_profile_sample(
                &q4_speed_profile, tick_ms
            );
            wheel_intent.requested_speed_left =
                chassis_motion_profile_scale_i16(
                    wheel_intent.requested_speed_left, q56_speed_scale
                );
            wheel_intent.requested_speed_right =
                chassis_motion_profile_scale_i16(
                    wheel_intent.requested_speed_right, q56_speed_scale
                );
            wheel_intent.duty_slew_step = HBALL_Q4_DUTY_SLEW_STEP;
            if (!q4_speed_profile.active && !q56_stop_profile_complete)
            {
                q56_stop_profile_complete = true;
                q56_stop_profile_complete_ms = tick_ms;
            }
        }
        error = follower_output.error;

        if (follower_output.reset_wheel_integrators)
        {
            wheel_control_reset_integrators(&wheel_control);
        }
        if (follower_output.lost_timeout)
        {
            finish_event_flags = HBALL_MISSION_CHASSIS_EVENT_STOPPED
                | HBALL_MISSION_CHASSIS_EVENT_LINE_LOST
                | HBALL_MISSION_CHASSIS_EVENT_LOCAL_FAULT;
            chassis_actuator_stop();
            chassis_actuator_set_wheel_speed(
                0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
            chassis_actuator_set_wheel_speed(
                0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
            chassis_actuator_disable();
            LCD_Fill(0, 48, 280, 100, BLACK);
            LCD_ShowString(
                4, 58, (const unsigned char *)"LOST STOP",
                RED, BLACK, 32, 0);
            break;
        }

        if (wheel_control_due(&wheel_control, tick_ms))
        {
            __disable_irq();
            current_left_count = Get_Encoder_countB;
            current_right_count = Get_Encoder_countA;
            __enable_irq();

            /*
             * WheelControl preserves the measured feed-forward, PID gains,
             * 100 ms normalization, integral bounds, and output slew.  This
             * caller owns only the atomic encoder snapshot and final actuation.
             */
            if (wheel_control_step(
                &wheel_control,
                tick_ms,
                current_left_count,
                current_right_count,
                &wheel_intent,
                &wheel_output
            ))
            {
                left_speed = wheel_output.measured_left_speed;
                right_speed = wheel_output.measured_right_speed;
                wheel_sample_valid = true;
                last_wheel_sample_ms = tick_ms;
                commanded_duty_left = wheel_output.duty_left;
                commanded_duty_right = wheel_output.duty_right;
                chassis_actuator_set_pwm((float)commanded_duty_left,
                                         (float)commanded_duty_right);

                if (log_index < LAP_LOG_SAMPLES)
                {
                    log_time_ms[log_index] = (uint16_t)elapsed_ms;
                    log_line_mask[log_index] = line_mask;
                    log_error[log_index] = (int8_t)error;
                    log_target_left[log_index] =
                        (uint8_t)wheel_intent.requested_speed_left;
                    log_target_right[log_index] =
                        (uint8_t)wheel_intent.requested_speed_right;
                    log_actual_left[log_index] = (int16_t)left_speed;
                    log_actual_right[log_index] = (int16_t)right_speed;
                    log_duty_left[log_index] = (uint8_t)commanded_duty_left;
                    log_duty_right[log_index] = (uint8_t)commanded_duty_right;
                    log_index++;
                }
                line_follower_ack_motion_applied(&line_follower);

                if (q56_stop_profile_complete)
                {
                    /* Wheel speed is encoder counts per 100 ms. */
                    if ((left_speed >= -HBALL_Q56_STOP_SPEED_THRESHOLD)
                        && (left_speed <= HBALL_Q56_STOP_SPEED_THRESHOLD)
                        && (right_speed >= -HBALL_Q56_STOP_SPEED_THRESHOLD)
                        && (right_speed <= HBALL_Q56_STOP_SPEED_THRESHOLD))
                    {
                        if (!q56_stop_settle_active)
                        {
                            q56_stop_settle_active = true;
                            q56_stop_settle_start_ms = tick_ms;
                        }
                        else if ((uint32_t)(tick_ms -
                                      q56_stop_settle_start_ms)
                                 >= HBALL_Q56_STOP_SETTLE_MS)
                        {
                            q56_stop_complete = true;
                        }
                    }
                    else
                    {
                        q56_stop_settle_active = false;
                    }
                }
            }
        }

        if (q56_stop_profile_complete)
        {
            /* The profile has reached zero; keep the chassis unpowered. */
            chassis_actuator_set_pwm(0.0F, 0.0F);
        }
        if (q56_stop_profile_complete && !q56_stop_complete
            && (!wheel_sample_valid
                || ((uint32_t)(tick_ms - last_wheel_sample_ms)
                    > HBALL_Q56_WHEEL_SAMPLE_STALE_MS))
            && ((uint32_t)(tick_ms - q56_stop_profile_complete_ms)
                >= HBALL_Q56_STOP_FALLBACK_MS))
        {
            /* Profile is already at zero; this bounds a missing sample. */
            q56_stop_complete = true;
        }
        if (q56_stop_complete)
        {
            finish_elapsed_ms = elapsed_ms;
            finish_event_flags = HBALL_MISSION_CHASSIS_EVENT_REACQUIRED_A;
            chassis_actuator_stop();
            chassis_actuator_set_wheel_speed(
                0.0f, (uint8_t)CHASSIS_WHEEL_LEFT);
            chassis_actuator_set_wheel_speed(
                0.0f, (uint8_t)CHASSIS_WHEEL_RIGHT);
            chassis_actuator_disable();
            LCD_Fill(0, 48, 280, 100, BLACK);
            LCD_ShowString(
                4, 58, (const unsigned char *)"LAP STOP",
                GREEN, BLACK, 32, 0);
            break;
        }

        competition_runtime_wait_ms(10U);
    }

    if (finish_elapsed_ms == 0U) {
        finish_elapsed_ms = (uint32_t)(tick_ms - run_start_ms);
    }
    format_lap_time(finish_elapsed_ms, time_text);
    LCD_ShowString(4, 106, (const unsigned char *)time_text, YELLOW, BLACK, 32, 0);
    telemetry_send_string("LAP_LOG_BEGIN,t,mask,e,tl,al,dl,tr,ar,dr\r\n");
    for (uint16_t i = 0U; i < log_index; i++)
    {
        snprintf((char *)uart_send, sizeof(uart_send),
                 "L,%u,%02X,%d,%u,%d,%u,%u,%d,%u\r\n",
                 (unsigned)log_time_ms[i],
                 (unsigned)log_line_mask[i],
                 (int)log_error[i],
                 (unsigned)log_target_left[i],
                 (int)log_actual_left[i],
                 (unsigned)log_duty_left[i],
                 (unsigned)log_target_right[i],
                 (int)log_actual_right[i],
                 (unsigned)log_duty_right[i]);
        telemetry_send_string((char *)uart_send);
    }
    telemetry_send_string("LAP_LOG_END\r\n");
    hball_can_mission_chassis_finish(finish_event_flags, tick_ms);
}
    


void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(TIMER_0_INST) != DL_TIMER_IIDX_ZERO)
    {
        return;
    }

    int32_t encoder_a_snapshot = Get_Encoder_countA;
    int32_t encoder_b_snapshot = Get_Encoder_countB;

    encoderA_cnt = encoder_a_snapshot - Get_Encoder_countA_LAST;
    Get_Encoder_countA_LAST = encoder_a_snapshot;
    encoderB_cnt = encoder_b_snapshot - Get_Encoder_countB_LAST;
    Get_Encoder_countB_LAST = encoder_b_snapshot;
}

void TIMER_1_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_1_INST) == DL_TIMER_IIDX_ZERO)
    {
        // 预留给后续固定周期任务。
    }
}



    // LCD_ShowString(1,32,(unsigned char *)"ENCOA:",BLUE,WHITE,32,0);
    // LCD_ShowString(1,64,(unsigned char *)"ENCOB:",BLUE,WHITE,32,0);
    // LCD_ShowString(1,96,(unsigned char *)"PWM1:",BLUE,WHITE,32,0);
    // LCD_ShowString(1,128,(unsigned char *)"PWM2:",BLUE,WHITE,32,0);
  



      // sprintf((char *)oled_buffer, "%d", encoderA_cnt);
      // LCD_ShowString(100,32,oled_buffer,BLUE,WHITE,32,0);
      // sprintf((char *)oled_buffer, "%d", encoderB_cnt);
      // LCD_ShowString(100,64,oled_buffer,BLUE,WHITE,32,0);
      // sprintf((char *)oled_buffer, "%.2f", RIGHT.Actual);
      // LCD_ShowString(100,128,oled_buffer,BLUE,WHITE,32,0);
      // float temp=wit_data.yaw;


//家居页面初始化
void ui_home_page(void)
{
    //关闭背光
	  LCD_BLK_Clr();

    //显示全屏背景颜色
    LCD_Fill(0,0,LCD_W,LCD_H,BLACK);
    //显示标题
    //disp_x_center(3,5,BLUE,16,(unsigned char *)"立创开发板");
    //显示副标题
    //disp_x_center(3+16+3,9,BLUE,16,(unsigned char *)"简易PID入门套件");

    //显示第一个按钮：定速
    //disp_string_rect(x, x_offset, y, y_offset, 2, 24, "定速", BLUE);
    //显示第二个按钮：定距
		//disp_string_rect(x2, x2_offset, y2, y2_offset, 2, 24, "定距", BLUE);

    //显示选择框
		//disp_select_box(40,80,65,80,10,5,WHITE);

    //打开背光
		LCD_BLK_Set();
}
