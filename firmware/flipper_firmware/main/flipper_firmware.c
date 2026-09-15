#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_err.h"

#define SERVO_LEDC_FREQ       50 
#define SERVO_LEDC_RES        LEDC_TIMER_14_BIT

#define LEFT_ARM_PIN          GPIO_NUM_18
#define RIGHT_ARM_PIN         GPIO_NUM_5

#define CH_LEFT               LEDC_CHANNEL_0
#define CH_RIGHT              LEDC_CHANNEL_1

// --- CALIBRATION LIMITS ---
// Left:  44° (Up)  <-> 132° (Down)
// Right: 124° (Up) <-> 40°  (Down)
#define LEFT_ARM_UP           44.0f
#define LEFT_ARM_DOWN         132.0f

#define RIGHT_ARM_UP          124.0f
#define RIGHT_ARM_DOWN        40.0f

// Starts at the exact middle of movement (50.0%)
static float arm_position_pct = 50.0f;

static inline uint32_t angle_to_ticks(float angle) {
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    float pulse_us = 500.0f + (angle / 180.0f) * 2000.0f;
    return (uint32_t)(pulse_us / 1.2207f);
}

static void apply_arm_sync(float pct) {
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    arm_position_pct = pct;

    float t = arm_position_pct / 100.0f;

    // Linear interpolation using your measured bounds
    float left_angle  = LEFT_ARM_DOWN  + t * (LEFT_ARM_UP - LEFT_ARM_DOWN);
    float right_angle = RIGHT_ARM_DOWN + t * (RIGHT_ARM_UP - RIGHT_ARM_DOWN);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_LEFT, angle_to_ticks(left_angle));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_LEFT);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_RIGHT, angle_to_ticks(right_angle));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_RIGHT);

    printf(">> Arm: %5.1f%% | Left (18): %5.1f deg | Right (5): %5.1f deg\n",
           arm_position_pct, left_angle, right_angle);
}

void app_main(void) {
    // 1. Install UART driver for instant keystrokes
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);

    // 2. LEDC 50Hz Timer Config
    ledc_timer_config_t timer_cfg = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = SERVO_LEDC_RES,
        .freq_hz          = SERVO_LEDC_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    // Calculate initial 50% midpoint angles
    float init_left_angle  = LEFT_ARM_DOWN  + 0.5f * (LEFT_ARM_UP - LEFT_ARM_DOWN);   // 88.0 deg
    float init_right_angle = RIGHT_ARM_DOWN + 0.5f * (RIGHT_ARM_UP - RIGHT_ARM_DOWN); // 82.0 deg

    // 3. Configure Left Servo Channel (GPIO 18) initialized to 50%
    ledc_channel_config_t left_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = CH_LEFT,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = LEFT_ARM_PIN,
        .duty       = angle_to_ticks(init_left_angle),
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&left_cfg));

    // 4. Configure Right Servo Channel (GPIO 5) initialized to 50%
    ledc_channel_config_t right_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = CH_RIGHT,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = RIGHT_ARM_PIN,
        .duty       = angle_to_ticks(init_right_angle),
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&right_cfg));

    printf("\n============================================\n");
    printf("     DUAL SYNC ARM CONTROLLER (18 & 5)      \n");
    printf("============================================\n");
    printf("Booted at 50.0%% Midpoint:\n");
    printf("  Left  (GPIO 18): %.1f deg\n", init_left_angle);
    printf("  Right (GPIO 5) : %.1f deg\n", init_right_angle);
    printf("--------------------------------------------\n");
    printf("Controls:\n");
    printf("  'w' -> Arm UP +2%%      's' -> Arm DOWN -2%%\n");
    printf("  'e' -> Arm UP +10%%     'd' -> Arm DOWN -10%%\n");
    printf("  '1' -> Move to 0%% (Rest / Down)\n");
    printf("  '2' -> Move to 50%% (Mid-height)\n");
    printf("  '3' -> Move to 100%% (Max Lift / Up)\n");
    printf("============================================\n\n");

    uint8_t rx_byte;
    while (1) {
        int len = uart_read_bytes(UART_NUM_0, &rx_byte, 1, pdMS_TO_TICKS(50));
        if (len > 0) {
            switch (rx_byte) {
                case 'w':
                case 'W':
                    apply_arm_sync(arm_position_pct + 2.0f);
                    break;
                case 's':
                case 'S':
                    apply_arm_sync(arm_position_pct - 2.0f);
                    break;
                case 'e':
                case 'E':
                    apply_arm_sync(arm_position_pct + 10.0f);
                    break;
                case 'd':
                case 'D':
                    apply_arm_sync(arm_position_pct - 10.0f);
                    break;
                case '1':
                    apply_arm_sync(0.0f);
                    break;
                case '2':
                    apply_arm_sync(50.0f);
                    break;
                case '3':
                    apply_arm_sync(100.0f);
                    break;
                default:
                    break;
            }
        }
    }
}