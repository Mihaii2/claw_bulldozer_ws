#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/string.h>

#define TAG "BULLDOZER_MICROROS"

// ==========================================
// === Router & micro-ROS Settings ==========
// ==========================================
#define TARGET_SSID         "Project"
#define TARGET_PASS         "formula1"
#define AGENT_IP            "192.168.1.127"
#define AGENT_PORT          "8888"

// ==========================================
// === Hardware Pinout Configuration ========
// ==========================================
#define MOTOR_L_IN1         GPIO_NUM_23
#define MOTOR_L_IN2         GPIO_NUM_22
#define MOTOR_R_IN3         GPIO_NUM_21
#define MOTOR_R_IN4         GPIO_NUM_19

#define CAM_SERVO_PIN       GPIO_NUM_4
#define ARM_L_SERVO_PIN     GPIO_NUM_18
#define ARM_R_SERVO_PIN     GPIO_NUM_5
#define CUP_SERVO_PIN       GPIO_NUM_17
#define CLAW_SERVO_PIN      GPIO_NUM_16

#define CH_SERVO_CAM        LEDC_CHANNEL_0
#define CH_SERVO_ARM_L      LEDC_CHANNEL_1
#define CH_SERVO_ARM_R      LEDC_CHANNEL_2
#define CH_SERVO_CUP        LEDC_CHANNEL_3
#define CH_SERVO_CLAW       LEDC_CHANNEL_4

#define CH_MOT_L_IN1        LEDC_CHANNEL_5
#define CH_MOT_L_IN2        LEDC_CHANNEL_6
#define CH_MOT_R_IN3        LEDC_CHANNEL_7

#define SERVO_TIMER         LEDC_TIMER_0
#define MOTOR_TIMER         LEDC_TIMER_1

#define SERVO_FREQ_HZ       50
#define SERVO_DUTY_RES      LEDC_TIMER_14_BIT

#define MOTOR_FREQ_HZ       1000
#define MOTOR_DUTY_RES      LEDC_TIMER_8_BIT
#define MOTOR_MAX_DUTY      255

// 2-Gear Configuration
#define DUTY_65_PCT         166
#define DUTY_100_PCT        255
static int current_drive_duty = DUTY_100_PCT;

#define DRIVE_TIMEOUT_US    160000 
static int64_t last_drive_cmd_time = 0;
static bool motors_active = false;

typedef struct {
    float arm_l_down;
    float arm_l_up;
    float arm_r_down;
    float arm_r_up;
} arm_calib_t;

static arm_calib_t arm_cal = {
    .arm_l_down = 138.0f,
    .arm_l_up   = 42.0f,
    .arm_r_down = 34.0f,
    .arm_r_up   = 122.0f
};

typedef struct {
    float target;
    float current;
    float max_step;
} servo_channel_state_t;

// Speed adjusted: Arms @ 75% (7.5f), Cup & Claw matching @ 10.0f
static servo_channel_state_t servo_cam   = { .target = 180.0f, .current = 180.0f, .max_step = 32.0f };
static servo_channel_state_t servo_arm_l = { .target = 99.6f,  .current = 99.6f,  .max_step = 7.5f };
static servo_channel_state_t servo_arm_r = { .target = 69.2f,  .current = 69.2f,  .max_step = 7.5f };
static servo_channel_state_t servo_cup   = { .target = 90.0f,  .current = 90.0f,  .max_step = 10.0f };
static servo_channel_state_t servo_claw  = { .target = 90.0f,  .current = 90.0f,  .max_step = 10.0f };

// User Memorized Intentions
static float intent_cam          = 180.0f;
static float intent_cup          = 90.0f;
static float intent_claw         = 90.0f;
static float current_arm_percent = 40.0f;

// Delay gate for restoration
static int64_t cam_clearance_ready_time = 0;

static portMUX_TYPE angle_mux = portMUX_INITIALIZER_UNLOCKED;

static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static inline uint32_t angle_to_ticks(float angle) {
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    float pulse_us = 500.0f + (angle / 180.0f) * 2000.0f;
    return (uint32_t)(pulse_us / 1.2207f);
}

static void apply_raw_servo(ledc_channel_t ch, float angle) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, angle_to_ticks(angle));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

static void set_motor_speeds(int left_pwm, int right_pwm) {
    if (left_pwm > 255) left_pwm = 255;
    if (left_pwm < -255) left_pwm = -255;
    if (right_pwm > 255) right_pwm = 255;
    if (right_pwm < -255) right_pwm = -255;

    // Left track
    if (left_pwm >= 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN1, left_pwm);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN2, 0);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN1, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN2, -left_pwm);
    }

    // Right track
    if (right_pwm >= 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_R_IN3, right_pwm);
        gpio_set_level(MOTOR_R_IN4, right_pwm > 0 ? 1 : 0);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_R_IN3, -right_pwm);
        gpio_set_level(MOTOR_R_IN4, 0);
    }

    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_MOT_R_IN3);

    if (left_pwm != 0 || right_pwm != 0) {
        last_drive_cmd_time = esp_timer_get_time();
        motors_active = true;
    } else {
        motors_active = false;
    }
}

static void drive_stop(void) {
    set_motor_speeds(0, 0);
}

// Drive Action Handler: Steering uses active gear on outside track and 0 on inside track
static void handle_drive_action(const char *action) {
    if (strcmp(action, "GEAR_1") == 0) {
        current_drive_duty = DUTY_65_PCT;
        ESP_LOGI(TAG, "Gear set to 1 (65%% PWM)");
        return;
    } else if (strcmp(action, "GEAR_2") == 0) {
        current_drive_duty = DUTY_100_PCT;
        ESP_LOGI(TAG, "Gear set to 2 (100%% PWM)");
        return;
    }

    if (strcmp(action, "FORWARD") == 0) {
        set_motor_speeds(current_drive_duty, current_drive_duty);
    } else if (strcmp(action, "REVERSE") == 0) {
        set_motor_speeds(-current_drive_duty, -current_drive_duty);
    } else if (strcmp(action, "SPIN_LEFT") == 0) {
        set_motor_speeds(MOTOR_MAX_DUTY, -MOTOR_MAX_DUTY);
    } else if (strcmp(action, "SPIN_RIGHT") == 0) {
        set_motor_speeds(-MOTOR_MAX_DUTY, MOTOR_MAX_DUTY);
    } else if (strcmp(action, "FWD_RIGHT") == 0) {
        // Outside (Left) at duty, Inside (Right) at 0
        set_motor_speeds(current_drive_duty, 0);
    } else if (strcmp(action, "FWD_LEFT") == 0) {
        // Outside (Right) at duty, Inside (Left) at 0
        set_motor_speeds(0, current_drive_duty);
    } else if (strcmp(action, "REV_RIGHT") == 0) {
        set_motor_speeds(-current_drive_duty, 0);
    } else if (strcmp(action, "REV_LEFT") == 0) {
        set_motor_speeds(0, -current_drive_duty);
    } else if (strcmp(action, "STOP") == 0) {
        drive_stop();
    }
}

// ============================================================
// === Piecewise Smooth Linear Interpolation Functions ========
// ============================================================
static inline float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

static float calc_smooth_claw_max(float arm_pct) {
    if (arm_pct <= 45.0f) return 180.0f;
    if (arm_pct <= 55.0f) return lerp(180.0f, 156.0f, (arm_pct - 45.0f) / (55.0f - 45.0f));
    if (arm_pct <= 60.0f) return lerp(156.0f, 151.0f, (arm_pct - 55.0f) / (60.0f - 55.0f));
    if (arm_pct <= 70.0f) return lerp(151.0f, 135.0f, (arm_pct - 60.0f) / (70.0f - 60.0f));
    if (arm_pct <= 80.0f) return lerp(135.0f, 120.0f, (arm_pct - 70.0f) / (80.0f - 70.0f));
    if (arm_pct <= 90.0f) return lerp(120.0f, 100.0f, (arm_pct - 80.0f) / (90.0f - 80.0f));
    if (arm_pct <= 100.0f) return lerp(100.0f, 90.0f, (arm_pct - 90.0f) / (100.0f - 90.0f));
    return 90.0f;
}

static float calc_smooth_cup_min(float arm_pct) {
    if (arm_pct <= 70.0f) return 0.0f;
    if (arm_pct <= 80.0f) return lerp(15.0f, 30.0f, (arm_pct - 70.0f) / 10.0f);
    if (arm_pct <= 90.0f) return lerp(30.0f, 55.0f, (arm_pct - 80.0f) / 10.0f);
    if (arm_pct <= 100.0f) return lerp(55.0f, 60.0f, (arm_pct - 90.0f) / 10.0f);
    return 60.0f;
}

static void calc_smooth_phone_intervals(float arm_pct, float *low_max, float *high_min) {
    if (arm_pct <= 70.0f) {
        *low_max = 180.0f;
        *high_min = 0.0f;
    } else if (arm_pct <= 80.0f) {
        float t = (arm_pct - 70.0f) / 10.0f;
        *low_max = lerp(115.0f, 100.0f, t);
        *high_min = 145.0f;
    } else if (arm_pct <= 90.0f) {
        float t = (arm_pct - 80.0f) / 10.0f;
        *low_max = lerp(100.0f, 95.0f, t);
        *high_min = 145.0f;
    } else {
        float t = (arm_pct - 90.0f) / 10.0f;
        if (t > 1.0f) t = 1.0f;
        *low_max = lerp(95.0f, 70.0f, t);
        *high_min = lerp(145.0f, 160.0f, t);
    }
}

static void govern_joint_limits_smooth(int64_t now_us) {
    float arm = current_arm_percent;

    // 1. Cup/Wrist Continuous Enforcement
    float min_cup = calc_smooth_cup_min(arm);
    servo_cup.target = (intent_cup < min_cup) ? min_cup : intent_cup;

    // 2. Claw Continuous Enforcement
    float max_claw = calc_smooth_claw_max(arm);
    servo_claw.target = (intent_claw > max_claw) ? max_claw : intent_claw;

    // 3. Phone / Cam Tilt Domain Protection & Delayed Auto-Restoration
    float low_max, high_min;
    calc_smooth_phone_intervals(arm, &low_max, &high_min);

    if (arm <= 70.0f) {
        if (cam_clearance_ready_time == 0) {
            cam_clearance_ready_time = now_us + 1000000;
        } else if (now_us >= cam_clearance_ready_time) {
            servo_cam.target = intent_cam;
        }
    } else {
        cam_clearance_ready_time = 0;
        bool in_lower = (servo_cam.current <= low_max);
        bool in_upper = (servo_cam.current >= high_min);

        if (in_lower) {
            servo_cam.target = (intent_cam > low_max) ? low_max : intent_cam;
        } else if (in_upper) {
            servo_cam.target = (intent_cam < high_min) ? high_min : intent_cam;
        }
    }
}

static void servo_smoothing_task(void *pvParameters) {
    while (1) {
        int64_t now = esp_timer_get_time();
        taskENTER_CRITICAL(&angle_mux);

        govern_joint_limits_smooth(now);

        if (fabsf(servo_cam.current - servo_cam.target) > 0.1f) {
            float delta = servo_cam.target - servo_cam.current;
            servo_cam.current += (fabsf(delta) <= servo_cam.max_step) ? delta : ((delta > 0) ? servo_cam.max_step : -servo_cam.max_step);
            apply_raw_servo(CH_SERVO_CAM, servo_cam.current);
        }

        if (fabsf(servo_arm_l.current - servo_arm_l.target) > 0.1f) {
            float delta = servo_arm_l.target - servo_arm_l.current;
            servo_arm_l.current += (fabsf(delta) <= servo_arm_l.max_step) ? delta : ((delta > 0) ? servo_arm_l.max_step : -servo_arm_l.max_step);
            apply_raw_servo(CH_SERVO_ARM_L, servo_arm_l.current);
        }

        if (fabsf(servo_arm_r.current - servo_arm_r.target) > 0.1f) {
            float delta = servo_arm_r.target - servo_arm_r.current;
            servo_arm_r.current += (fabsf(delta) <= servo_arm_r.max_step) ? delta : ((delta > 0) ? servo_arm_r.max_step : -servo_arm_r.max_step);
            apply_raw_servo(CH_SERVO_ARM_R, servo_arm_r.current);
        }

        if (fabsf(servo_cup.current - servo_cup.target) > 0.1f) {
            float delta = servo_cup.target - servo_cup.current;
            servo_cup.current += (fabsf(delta) <= servo_cup.max_step) ? delta : ((delta > 0) ? servo_cup.max_step : -servo_cup.max_step);
            apply_raw_servo(CH_SERVO_CUP, servo_cup.current);
        }

        if (fabsf(servo_claw.current - servo_claw.target) > 0.1f) {
            float delta = servo_claw.target - servo_claw.current;
            servo_claw.current += (fabsf(delta) <= servo_claw.max_step) ? delta : ((delta > 0) ? servo_claw.max_step : -servo_claw.max_step);
            apply_raw_servo(CH_SERVO_CLAW, servo_claw.current);
        }

        taskEXIT_CRITICAL(&angle_mux);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static bool check_and_apply_arm_percent(float target_pct) {
    if (target_pct < 0.0f)   target_pct = 0.0f;
    if (target_pct > 100.0f) target_pct = 100.0f;

    taskENTER_CRITICAL(&angle_mux);
    float cur_cam = servo_cam.current;

    // Hard ceiling: if phone is in the danger zone, arm CANNOT go above 70%
    if (cur_cam >= 115.0f && cur_cam <= 145.0f) {
        if (target_pct > 70.0f) {
            target_pct = 70.0f; // CLAMP, NEVER REJECT!
        }
    }

    // Corridor ceiling: if phone is in the interpolated forbidden band, clamp to 70%
    if (target_pct > 70.0f) {
        float low_max, high_min;
        calc_smooth_phone_intervals(target_pct, &low_max, &high_min);
        if (cur_cam > low_max && cur_cam < high_min) {
            target_pct = 70.0f; // CLAMP TO CEILING
        }
    }

    // Always update current_arm_percent to the clamped value
    current_arm_percent = target_pct;
    float p = current_arm_percent / 100.0f;
    servo_arm_l.target = arm_cal.arm_l_down - p * (arm_cal.arm_l_down - arm_cal.arm_l_up);
    servo_arm_r.target = arm_cal.arm_r_down + p * (arm_cal.arm_r_up - arm_cal.arm_r_down);

    taskEXIT_CRITICAL(&angle_mux);
    return true;
}

static bool check_and_apply_cam_intent(float target_deg) {
    if (target_deg < 0.0f)   target_deg = 0.0f;
    if (target_deg > 180.0f) target_deg = 180.0f;

    taskENTER_CRITICAL(&angle_mux);
    float arm = current_arm_percent;
    float cur_cam = servo_cam.current;

    // If arm is above 70%, clamp target_deg to the allowed side
    if (arm > 70.0f) {
        float low_max, high_min;
        calc_smooth_phone_intervals(arm, &low_max, &high_min);

        bool in_lower = (cur_cam <= low_max);
        if (in_lower) {
            if (target_deg > low_max) target_deg = low_max; // CLAMP TO LOWER SAFE BOUND
        } else {
            if (target_deg < high_min) target_deg = high_min; // CLAMP TO UPPER SAFE BOUND
        }
    }

    intent_cam = target_deg;
    servo_cam.target = target_deg;

    taskEXIT_CRITICAL(&angle_mux);
    return true;
}

static void init_actuators(void) {
    ledc_timer_config_t servo_t = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = SERVO_TIMER,
        .duty_resolution  = SERVO_DUTY_RES,
        .freq_hz          = SERVO_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&servo_t));

    ledc_timer_config_t motor_t = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = MOTOR_TIMER,
        .duty_resolution  = MOTOR_DUTY_RES,
        .freq_hz          = MOTOR_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&motor_t));

    float start_arm_l = arm_cal.arm_l_down - 0.40f * (arm_cal.arm_l_down - arm_cal.arm_l_up);
    float start_arm_r = arm_cal.arm_r_down + 0.40f * (arm_cal.arm_r_up - arm_cal.arm_r_down);

    ledc_channel_config_t servos[] = {
        { .gpio_num = CAM_SERVO_PIN,   .channel = CH_SERVO_CAM,   .duty = angle_to_ticks(180.0f) },
        { .gpio_num = ARM_L_SERVO_PIN, .channel = CH_SERVO_ARM_L, .duty = angle_to_ticks(start_arm_l) },
        { .gpio_num = ARM_R_SERVO_PIN, .channel = CH_SERVO_ARM_R, .duty = angle_to_ticks(start_arm_r) },
        { .gpio_num = CUP_SERVO_PIN,   .channel = CH_SERVO_CUP,   .duty = angle_to_ticks(90.0f) },
        { .gpio_num = CLAW_SERVO_PIN,  .channel = CH_SERVO_CLAW,  .duty = angle_to_ticks(90.0f) },
    };

    for (int i = 0; i < 5; i++) {
        servos[i].speed_mode = LEDC_LOW_SPEED_MODE;
        servos[i].timer_sel  = SERVO_TIMER;
        servos[i].intr_type  = LEDC_INTR_DISABLE;
        servos[i].hpoint     = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&servos[i]));
    }

    ledc_channel_config_t motors[] = {
        { .gpio_num = MOTOR_L_IN1, .channel = CH_MOT_L_IN1 },
        { .gpio_num = MOTOR_L_IN2, .channel = CH_MOT_L_IN2 },
        { .gpio_num = MOTOR_R_IN3, .channel = CH_MOT_R_IN3 }
    };
    for (int i = 0; i < 3; i++) {
        motors[i].speed_mode = LEDC_LOW_SPEED_MODE;
        motors[i].timer_sel  = MOTOR_TIMER;
        motors[i].intr_type  = LEDC_INTR_DISABLE;
        motors[i].duty       = 0;
        motors[i].hpoint     = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&motors[i]));
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_R_IN4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(MOTOR_R_IN4, 0);

    apply_raw_servo(CH_SERVO_CAM, 180.0f);
    apply_raw_servo(CH_SERVO_ARM_L, start_arm_l);
    apply_raw_servo(CH_SERVO_ARM_R, start_arm_r);
    apply_raw_servo(CH_SERVO_CUP, 90.0f);
    apply_raw_servo(CH_SERVO_CLAW, 90.0f);
}

// ----------------------------------------------------
// DRIVE CALLBACK: /bulldozer/drive_cmd
// ----------------------------------------------------
void drive_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    if (!msg || !msg->data.data) return;
    handle_drive_action(msg->data.data);
}

// ----------------------------------------------------
// ARM / MANIPULATOR CALLBACK: /bulldozer/arm_cmd
// ----------------------------------------------------
void arm_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    if (!msg || !msg->data.data) return;
    const char *cmd = msg->data.data;

    if (strcmp(cmd, "ARM_UP") == 0) {
        check_and_apply_arm_percent(current_arm_percent + 2.25f);
    } else if (strcmp(cmd, "ARM_DOWN") == 0) {
        check_and_apply_arm_percent(current_arm_percent - 2.25f);
    } else if (strcmp(cmd, "CUP_UP") == 0) {
        taskENTER_CRITICAL(&angle_mux);
        intent_cup = fmaxf(0.0f, intent_cup - 3.5f);
        taskEXIT_CRITICAL(&angle_mux);
    } else if (strcmp(cmd, "CUP_DOWN") == 0) {
        taskENTER_CRITICAL(&angle_mux);
        intent_cup = fminf(180.0f, intent_cup + 3.5f);
        taskEXIT_CRITICAL(&angle_mux);
    } else if (strcmp(cmd, "CLAW_CLOSE") == 0) {
        taskENTER_CRITICAL(&angle_mux);
        intent_claw = fminf(180.0f, intent_claw + 3.5f);
        taskEXIT_CRITICAL(&angle_mux);
    } else if (strcmp(cmd, "CLAW_OPEN") == 0) {
        taskENTER_CRITICAL(&angle_mux);
        intent_claw = fmaxf(0.0f, intent_claw - 3.5f);
        taskEXIT_CRITICAL(&angle_mux);
    } else if (strcmp(cmd, "CAM_UP") == 0) {
        check_and_apply_cam_intent(intent_cam + 8.0f);
    } else if (strcmp(cmd, "CAM_DOWN") == 0) {
        check_and_apply_cam_intent(intent_cam - 8.0f);
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = TARGET_SSID,
            .password = TARGET_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_max_tx_power(78);
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "Connecting to Wi-Fi: %s...", TARGET_SSID);
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Wi-Fi Connected successfully.");
}

void safety_watchdog_task(void *pvParameters) {
    while (1) {
        if (motors_active && (esp_timer_get_time() - last_drive_cmd_time > DRIVE_TIMEOUT_US)) {
            drive_stop();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void micro_ros_task(void * arg) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options;
    rcl_node_t node;
    
    rcl_subscription_t cmd_drive_sub;
    rcl_subscription_t cmd_arm_sub;
    rclc_executor_t executor;
    
    std_msgs__msg__String drive_msg;
    std_msgs__msg__String arm_msg;
    char drive_buffer[32];
    char arm_buffer[32];

    drive_msg.data.data = drive_buffer;
    drive_msg.data.capacity = sizeof(drive_buffer);
    drive_msg.data.size = 0;

    arm_msg.data.data = arm_buffer;
    arm_msg.data.capacity = sizeof(arm_buffer);
    arm_msg.data.size = 0;

    while (1) {
        xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

        init_options = rcl_get_zero_initialized_init_options();
        if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
        rmw_uros_options_set_udp_address(AGENT_IP, AGENT_PORT, rmw_options);

        ESP_LOGI(TAG, "Connecting to micro-ROS agent at %s:%s...", AGENT_IP, AGENT_PORT);
        while (rmw_uros_ping_agent_options(100, 2, rmw_options) != RMW_RET_OK) {
            if (!(xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT)) break;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        if (!(xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT)) {
            rcl_init_options_fini(&init_options);
            continue;
        }

        ESP_LOGI(TAG, "Connected to Agent. Setting up Node...");
        ESP_ERROR_CHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
        ESP_ERROR_CHECK(rclc_node_init_default(&node, "bulldozer_hardware_brain", "", &support));

        ESP_ERROR_CHECK(rclc_subscription_init_default(
            &cmd_drive_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/bulldozer/drive_cmd"));

        ESP_ERROR_CHECK(rclc_subscription_init_default(
            &cmd_arm_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/bulldozer/arm_cmd"));

        ESP_ERROR_CHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
        ESP_ERROR_CHECK(rclc_executor_add_subscription(&executor, &cmd_drive_sub, &drive_msg, &drive_callback, ON_NEW_DATA));
        ESP_ERROR_CHECK(rclc_executor_add_subscription(&executor, &cmd_arm_sub, &arm_msg, &arm_callback, ON_NEW_DATA));

        ESP_LOGI(TAG, "micro-ROS Ready: /bulldozer/drive_cmd & /bulldozer/arm_cmd active");

        uint32_t ping_counter = 0;
        while (1) {
            rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) break;

            if (++ping_counter >= 200) {
                ping_counter = 0;
                if (!(xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT) ||
                    rmw_uros_ping_agent_options(100, 1, rmw_options) != RMW_RET_OK) {
                    ESP_LOGW(TAG, "Heartbeat lost, resetting transport...");
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        drive_stop();
        rclc_executor_fini(&executor);
        rcl_subscription_fini(&cmd_drive_sub, &node);
        rcl_subscription_fini(&cmd_arm_sub, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        rcl_init_options_fini(&init_options);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_actuators();
    wifi_init_sta();

    xTaskCreatePinnedToCore(servo_smoothing_task, "servo_slew", 2048, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(safety_watchdog_task, "watchdog", 2048, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(micro_ros_task, "uros_task", 4096 * 4, NULL, 5, NULL, 0);
}