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

#define TAG "BULLDOZER_BRAIN"

// ==========================================
// === Wi-Fi & micro-ROS Settings ===
// ==========================================
#define TARGET_SSID         "laptop_flipper"
#define TARGET_PASS         "mafiosu123"
#define AGENT_IP            "10.42.0.1"
#define AGENT_PORT          "8888"

// ==========================================
// === Hardware Pinout Configuration ===
// ==========================================
// TC1508 DC Motors
#define MOTOR_R_IN1         GPIO_NUM_23
#define MOTOR_R_IN2         GPIO_NUM_22
#define MOTOR_L_IN3         GPIO_NUM_21
#define MOTOR_L_IN4         GPIO_NUM_19

// MG995 Servos
#define LEFT_ARM_PIN        GPIO_NUM_18
#define RIGHT_ARM_PIN       GPIO_NUM_5
#define CUP_TILT_PIN        GPIO_NUM_17
#define CLAW_PIN            GPIO_NUM_16

// LEDC Allocations
#define CH_SERVO_ARM_L      LEDC_CHANNEL_0
#define CH_SERVO_ARM_R      LEDC_CHANNEL_1
#define CH_SERVO_CUP        LEDC_CHANNEL_2
#define CH_SERVO_CLAW       LEDC_CHANNEL_3

#define CH_MOT_R_IN1        LEDC_CHANNEL_4
#define CH_MOT_R_IN2        LEDC_CHANNEL_5
#define CH_MOT_L_IN3        LEDC_CHANNEL_6
#define CH_MOT_L_IN4        LEDC_CHANNEL_7

#define SERVO_TIMER         LEDC_TIMER_0
#define MOTOR_TIMER         LEDC_TIMER_1

#define SERVO_FREQ_HZ       50
#define SERVO_DUTY_RES      LEDC_TIMER_14_BIT

#define MOTOR_FREQ_HZ       1000
#define MOTOR_DUTY_RES      LEDC_TIMER_8_BIT
#define MOTOR_MAX_DUTY      255

// Calibrated Travel Limits
#define LEFT_ARM_UP         46.0f
#define LEFT_ARM_DOWN       132.0f
#define RIGHT_ARM_UP        124.0f
#define RIGHT_ARM_DOWN      42.0f

#define CUP_MIN             20.0f
#define CUP_MAX             160.0f
#define CLAW_MIN            20.0f
#define CLAW_MAX            160.0f

// Safety Watchdog
#define DRIVE_TIMEOUT_US    350000 
static int64_t last_drive_cmd_time = 0;
static bool motors_active = false;

// Global Actuator Positions
static float arm_pct    = 90.0f;  // Boots high
static float cup_angle  = 120.0f; 
static float claw_angle = 90.0f;  

// Current active gear: 1, 2, or 3
static int current_gear = 1;

// Fixed PWM Duty Definitions
#define DUTY_55_PCT         140  // ~55% PWM
#define DUTY_75_PCT         191  // ~75% PWM
#define DUTY_100_PCT        255  // 100% PWM

static EventGroupHandle_t wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;

static inline uint32_t angle_to_ticks(float angle) {
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    float pulse_us = 500.0f + (angle / 180.0f) * 2000.0f;
    return (uint32_t)(pulse_us / 1.2207f);
}

// Low-Level Motor Control: Signed PWM (-255 to +255)
static void set_motor_speeds(int left_pwm, int right_pwm) {
    if (left_pwm > 255) left_pwm = 255;
    if (left_pwm < -255) left_pwm = -255;
    if (right_pwm > 255) right_pwm = 255;
    if (right_pwm < -255) right_pwm = -255;

    // Right track: IN1 / IN2
    if (right_pwm >= 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_R_IN1, right_pwm);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_R_IN2, 0);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_R_IN1, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_R_IN2, -right_pwm);
    }

    // Left track: IN3 / IN4
    if (left_pwm >= 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN3, left_pwm);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN4, 0);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN3, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN4, -left_pwm);
    }

    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_MOT_R_IN1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_MOT_R_IN2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN3);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN4);

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

// Arm, Cup & Claw helpers
static void apply_arm_sync(float pct) {
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    arm_pct = pct;

    float t = arm_pct / 100.0f;
    float l_deg = LEFT_ARM_DOWN  + t * (LEFT_ARM_UP - LEFT_ARM_DOWN);
    float r_deg = RIGHT_ARM_DOWN + t * (RIGHT_ARM_UP - RIGHT_ARM_DOWN);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_SERVO_ARM_L, angle_to_ticks(l_deg));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_SERVO_ARM_L);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_SERVO_ARM_R, angle_to_ticks(r_deg));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_SERVO_ARM_R);
}

static void apply_cup(float deg) {
    if (deg < CUP_MIN) deg = CUP_MIN;
    if (deg > CUP_MAX) deg = CUP_MAX;
    cup_angle = deg;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_SERVO_CUP, angle_to_ticks(cup_angle));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_SERVO_CUP);
}

static void apply_claw(float deg) {
    if (deg < CLAW_MIN) deg = CLAW_MIN;
    if (deg > CLAW_MAX) deg = CLAW_MAX;
    claw_angle = deg;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_SERVO_CLAW, angle_to_ticks(claw_angle));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_SERVO_CLAW);
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

    float init_l = LEFT_ARM_DOWN  + 0.9f * (LEFT_ARM_UP - LEFT_ARM_DOWN);
    float init_r = RIGHT_ARM_DOWN + 0.9f * (RIGHT_ARM_UP - RIGHT_ARM_DOWN);

    ledc_channel_config_t servos[] = {
        { .gpio_num = LEFT_ARM_PIN,  .channel = CH_SERVO_ARM_L, .duty = angle_to_ticks(init_l) },
        { .gpio_num = RIGHT_ARM_PIN, .channel = CH_SERVO_ARM_R, .duty = angle_to_ticks(init_r) },
        { .gpio_num = CUP_TILT_PIN,  .channel = CH_SERVO_CUP,   .duty = angle_to_ticks(cup_angle) },
        { .gpio_num = CLAW_PIN,      .channel = CH_SERVO_CLAW,  .duty = angle_to_ticks(claw_angle) },
    };
    for (int i = 0; i < 4; i++) {
        servos[i].speed_mode = LEDC_LOW_SPEED_MODE;
        servos[i].timer_sel  = SERVO_TIMER;
        servos[i].intr_type  = LEDC_INTR_DISABLE;
        servos[i].hpoint     = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&servos[i]));
    }

    ledc_channel_config_t motors[] = {
        { .gpio_num = MOTOR_R_IN1, .channel = CH_MOT_R_IN1 },
        { .gpio_num = MOTOR_R_IN2, .channel = CH_MOT_R_IN2 },
        { .gpio_num = MOTOR_L_IN3, .channel = CH_MOT_L_IN3 },
        { .gpio_num = MOTOR_L_IN4, .channel = CH_MOT_L_IN4 },
    };
    for (int i = 0; i < 4; i++) {
        motors[i].speed_mode = LEDC_LOW_SPEED_MODE;
        motors[i].timer_sel  = MOTOR_TIMER;
        motors[i].intr_type  = LEDC_INTR_DISABLE;
        motors[i].duty       = 0;
        motors[i].hpoint     = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&motors[i]));
    }
}

// Wi-Fi Configuration
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
            .pmf_cfg = { .capable = true, .required = false },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "Connecting to Wi-Fi: %s...", TARGET_SSID);
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Wi-Fi Connected successfully.");
}

// ----------------------------------------------------
// DRIVE CALLBACK: Locomotion only (/bulldozer/drive_cmd)
// ----------------------------------------------------
void drive_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    if (!msg || !msg->data.data) return;
    const char *cmd = msg->data.data;

    // Gear setting
    if (strcmp(cmd, "GEAR_1") == 0)      current_gear = 1;
    else if (strcmp(cmd, "GEAR_2") == 0) current_gear = 2;
    else if (strcmp(cmd, "GEAR_3") == 0) current_gear = 3;

    int straight_duty = DUTY_55_PCT;
    if (current_gear == 2) straight_duty = DUTY_75_PCT;
    else if (current_gear == 3) straight_duty = DUTY_100_PCT;

    // 100% outside, 0% inside on turns
    const int turn_outside_duty = DUTY_100_PCT;
    const int turn_inside_duty  = 0;

    if (strcmp(cmd, "FORWARD") == 0) {
        set_motor_speeds(straight_duty, straight_duty);
    } else if (strcmp(cmd, "REVERSE") == 0) {
        set_motor_speeds(-straight_duty, -straight_duty);
    } else if (strcmp(cmd, "SPIN_LEFT") == 0) {
        set_motor_speeds(-MOTOR_MAX_DUTY, MOTOR_MAX_DUTY);
    } else if (strcmp(cmd, "SPIN_RIGHT") == 0) {
        set_motor_speeds(MOTOR_MAX_DUTY, -MOTOR_MAX_DUTY);
    } else if (strcmp(cmd, "FWD_LEFT") == 0) {
        set_motor_speeds(turn_inside_duty, turn_outside_duty);
    } else if (strcmp(cmd, "FWD_RIGHT") == 0) {
        set_motor_speeds(turn_outside_duty, turn_inside_duty);
    } else if (strcmp(cmd, "REV_LEFT") == 0) {
        set_motor_speeds(-turn_inside_duty, -turn_outside_duty);
    } else if (strcmp(cmd, "REV_RIGHT") == 0) {
        set_motor_speeds(-turn_outside_duty, -turn_inside_duty);
    } else if (strcmp(cmd, "STOP") == 0) {
        drive_stop();
    }
}

// ----------------------------------------------------
// ARM CALLBACK: Manipulator only (/bulldozer/arm_cmd)
// ----------------------------------------------------
void arm_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    if (!msg || !msg->data.data) return;
    const char *cmd = msg->data.data;

    if (strcmp(cmd, "ARM_UP") == 0)          apply_arm_sync(arm_pct + 5.0f);
    else if (strcmp(cmd, "ARM_DOWN") == 0)   apply_arm_sync(arm_pct - 5.0f);
    else if (strcmp(cmd, "CUP_UP") == 0)     apply_cup(cup_angle + 5.0f);
    else if (strcmp(cmd, "CUP_DOWN") == 0)   apply_cup(cup_angle - 5.0f);
    else if (strcmp(cmd, "CLAW_OPEN") == 0)  apply_claw(claw_angle + 5.0f);
    else if (strcmp(cmd, "CLAW_CLOSE") == 0) apply_claw(claw_angle - 5.0f);
}

void safety_watchdog_task(void *pvParameters) {
    while (1) {
        if (motors_active && (esp_timer_get_time() - last_drive_cmd_time > DRIVE_TIMEOUT_US)) {
            drive_stop();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void micro_ros_task(void * arg) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options;
    rcl_node_t node;
    
    // Decoupled subscriptions
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

        // Initialize both independent subscribers
        ESP_ERROR_CHECK(rclc_subscription_init_default(
            &cmd_drive_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/bulldozer/drive_cmd"));

        ESP_ERROR_CHECK(rclc_subscription_init_default(
            &cmd_arm_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/bulldozer/arm_cmd"));

        // Executor handles 2 subscription handles
        ESP_ERROR_CHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
        ESP_ERROR_CHECK(rclc_executor_add_subscription(&executor, &cmd_drive_sub, &drive_msg, &drive_callback, ON_NEW_DATA));
        ESP_ERROR_CHECK(rclc_executor_add_subscription(&executor, &cmd_arm_sub, &arm_msg, &arm_callback, ON_NEW_DATA));

        ESP_LOGI(TAG, "Decoupled listeners active: /bulldozer/drive_cmd & /bulldozer/arm_cmd");

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

    xTaskCreatePinnedToCore(safety_watchdog_task, "watchdog", 2048, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(micro_ros_task, "uros_task", 4096 * 4, NULL, 5, NULL, 0);
}