#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_http_server.h"
#include "esp_netif.h"

#define TAG "BULLDOZER_CTRL"

// ============================================================
// === Date HOTSPOT Redmi Note 13 Pro =========================
// ============================================================
#define HOTSPOT_SSID        "My_Redmi"
#define HOTSPOT_PASS        "formula1"

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// ==========================================
// === Hardware Pinout Configuration ========
// ==========================================
#define MOTOR_L_IN1         GPIO_NUM_23
#define MOTOR_L_IN2         GPIO_NUM_22
#define MOTOR_R_IN3         GPIO_NUM_21
#define MOTOR_R_IN4         GPIO_NUM_19

#define LEFT_ARM_PIN        GPIO_NUM_18
#define RIGHT_ARM_PIN       GPIO_NUM_5
#define CUP_TILT_PIN        GPIO_NUM_17
#define CLAW_PIN            GPIO_NUM_16

#define CH_SERVO_ARM_L      LEDC_CHANNEL_0
#define CH_SERVO_ARM_R      LEDC_CHANNEL_1
#define CH_SERVO_CUP        LEDC_CHANNEL_2
#define CH_SERVO_CLAW       LEDC_CHANNEL_3

#define CH_MOT_L_IN1        LEDC_CHANNEL_4
#define CH_MOT_L_IN2        LEDC_CHANNEL_5
#define CH_MOT_R_IN3        LEDC_CHANNEL_6
#define CH_MOT_R_IN4        LEDC_CHANNEL_7

#define SERVO_TIMER         LEDC_TIMER_0
#define MOTOR_TIMER         LEDC_TIMER_1

#define SERVO_FREQ_HZ       50
#define SERVO_DUTY_RES      LEDC_TIMER_14_BIT

#define MOTOR_FREQ_HZ       1000
#define MOTOR_DUTY_RES      LEDC_TIMER_8_BIT
#define MOTOR_MAX_DUTY      255

#define LEFT_ARM_UP         46.0f
#define LEFT_ARM_DOWN       132.0f
#define RIGHT_ARM_UP        124.0f
#define RIGHT_ARM_DOWN      42.0f

#define CUP_MIN             20.0f
#define CUP_MAX             160.0f
#define CLAW_MIN            20.0f
#define CLAW_MAX            160.0f

#define DRIVE_TIMEOUT_US    400000 
static int64_t last_drive_cmd_time = 0;
static bool motors_active = false;

static float arm_pct    = 90.0f;
static float cup_angle  = 120.0f; 
static float claw_angle = 90.0f;  

static inline uint32_t angle_to_ticks(float angle) {
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    float pulse_us = 500.0f + (angle / 180.0f) * 2000.0f;
    return (uint32_t)(pulse_us / 1.2207f);
}

static void set_motor_speeds(int left_pwm, int right_pwm) {
    if (left_pwm > 255) left_pwm = 255;
    if (left_pwm < -255) left_pwm = -255;
    if (right_pwm > 255) right_pwm = 255;
    if (right_pwm < -255) right_pwm = -255;

    if (left_pwm >= 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN1, left_pwm);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN2, 0);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN1, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN2, -left_pwm);
    }

    if (right_pwm >= 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_R_IN3, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_R_IN4, right_pwm);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_R_IN3, -right_pwm);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_R_IN4, 0);
    }

    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_MOT_R_IN3);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_MOT_R_IN4);

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

    ledc_channel_config_t servos[] = {
        { .gpio_num = LEFT_ARM_PIN,  .channel = CH_SERVO_ARM_L, .duty = 0 },
        { .gpio_num = RIGHT_ARM_PIN, .channel = CH_SERVO_ARM_R, .duty = 0 },
        { .gpio_num = CUP_TILT_PIN,  .channel = CH_SERVO_CUP,   .duty = 0 },
        { .gpio_num = CLAW_PIN,      .channel = CH_SERVO_CLAW,  .duty = 0 },
    };
    for (int i = 0; i < 4; i++) {
        servos[i].speed_mode = LEDC_LOW_SPEED_MODE;
        servos[i].timer_sel  = SERVO_TIMER;
        servos[i].intr_type  = LEDC_INTR_DISABLE;
        servos[i].hpoint     = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&servos[i]));
    }

    ledc_channel_config_t motors[] = {
        { .gpio_num = MOTOR_L_IN1, .channel = CH_MOT_L_IN1 },
        { .gpio_num = MOTOR_L_IN2, .channel = CH_MOT_L_IN2 },
        { .gpio_num = MOTOR_R_IN3, .channel = CH_MOT_R_IN3 },
        { .gpio_num = MOTOR_R_IN4, .channel = CH_MOT_R_IN4 },
    };
    for (int i = 0; i < 4; i++) {
        motors[i].speed_mode = LEDC_LOW_SPEED_MODE;
        motors[i].timer_sel  = MOTOR_TIMER;
        motors[i].intr_type  = LEDC_INTR_DISABLE;
        motors[i].duty       = 0;
        motors[i].hpoint     = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&motors[i]));
    }

    // Force actuators directly into ready pose at boot (even before Wi-Fi connects)
    apply_arm_sync(90.0f);
    apply_cup(120.0f);
    apply_claw(90.0f);
}

// ==========================================
// === Web UI HTML ==========================
// ==========================================
static const char INDEX_HTML[] = 
"<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0,user-scalable=no'>"
"<title>Bulldozer Teleop</title><style>"
"body{background:#1a1a1a;color:#eee;font-family:sans-serif;text-align:center;margin:0;padding:10px;user-select:none;touch-action:manipulation;}"
"h2{margin:5px 0 15px 0;color:#f39c12;}"
".grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:300px;margin:0 auto 20px auto;}"
"button{background:#333;color:#fff;border:2px solid #555;padding:20px;font-size:18px;border-radius:12px;font-weight:bold;cursor:pointer;}"
"button:active{background:#f39c12;color:#000;}"
".arm-grid{display:grid;grid-template-columns:repeat(2,1fr);gap:10px;max-width:300px;margin:0 auto;}"
".btn-arm{background:#2c3e50;padding:15px;}"
".btn-stop{background:#c0392b;border-color:#e74c3c;}"
".hint{color:#888;font-size:13px;margin-bottom:15px;}"
"</style></head><body>"
"<h2>🚜 Bulldozer Control</h2>"
"<div class='hint'>Use buttons or WASD / Arrow Keys</div>"
"<div class='grid'>"
"<div></div><button onpointerdown=\"startHold('FORWARD')\" onpointerup=\"stopHold()\" onpointerleave=\"stopHold()\">▲</button><div></div>"
"<button onpointerdown=\"startHold('SPIN_LEFT')\" onpointerup=\"stopHold()\" onpointerleave=\"stopHold()\">◄</button>"
"<button class='btn-stop' onclick=\"stopHold()\">■</button>"
"<button onpointerdown=\"startHold('SPIN_RIGHT')\" onpointerup=\"stopHold()\" onpointerleave=\"stopHold()\">►</button>"
"<div></div><button onpointerdown=\"startHold('REVERSE')\" onpointerup=\"stopHold()\" onpointerleave=\"stopHold()\">▼</button><div></div>"
"</div>"
"<h3>Manipulator</h3>"
"<div class='arm-grid'>"
"<button class='btn-arm' onclick=\"send('ARM_UP')\">Arm ▲</button><button class='btn-arm' onclick=\"send('ARM_DOWN')\">Arm ▼</button>"
"<button class='btn-arm' onclick=\"send('CUP_UP')\">Cup ▲</button><button class='btn-arm' onclick=\"send('CUP_DOWN')\">Cup ▼</button>"
"<button class='btn-arm' onclick=\"send('CLAW_OPEN')\">Claw ◄►</button><button class='btn-arm' onclick=\"send('CLAW_CLOSE')\">Claw ►◄</button>"
"</div>"
"<script>"
"let driveInterval = null;"
"let activeCmd = null;"

"function send(cmd){fetch('/cmd?action='+cmd);}"

"function startHold(cmd){"
"  if(activeCmd === cmd) return;"
"  stopHold();"
"  activeCmd = cmd;"
"  send(cmd);"
"  driveInterval = setInterval(()=>{ send(cmd); }, 100);"
"}"

"function stopHold(){"
"  if(driveInterval){"
"    clearInterval(driveInterval);"
"    driveInterval = null;"
"  }"
"  if(activeCmd !== null){"
"    activeCmd = null;"
"    send('STOP');"
"  }"
"}"

"const keyMap = {"
"  'KeyW':'FORWARD','ArrowUp':'FORWARD',"
"  'KeyS':'REVERSE','ArrowDown':'REVERSE',"
"  'KeyA':'SPIN_LEFT','ArrowLeft':'SPIN_LEFT',"
"  'KeyD':'SPIN_RIGHT','ArrowRight':'SPIN_RIGHT'"
"};"

"window.addEventListener('keydown', (e)=>{"
"  if(e.repeat) return;"
"  const cmd = keyMap[e.code];"
"  if(cmd) startHold(cmd);"
"});"

"window.addEventListener('keyup', (e)=>{"
"  const cmd = keyMap[e.code];"
"  if(cmd && activeCmd === cmd) stopHold();"
"});"

"window.addEventListener('blur', ()=> stopHold());"
"</script></body></html>";

static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t cmd_handler(httpd_req_t *req) {
    char buf[64];
    size_t buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1 && buf_len <= sizeof(buf)) {
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            char action[32];
            if (httpd_query_key_value(buf, "action", action, sizeof(action)) == ESP_OK) {
                if (strcmp(action, "FORWARD") == 0) set_motor_speeds(MOTOR_MAX_DUTY, MOTOR_MAX_DUTY);
                else if (strcmp(action, "REVERSE") == 0) set_motor_speeds(-MOTOR_MAX_DUTY, -MOTOR_MAX_DUTY);
                else if (strcmp(action, "SPIN_LEFT") == 0) set_motor_speeds(MOTOR_MAX_DUTY, -MOTOR_MAX_DUTY);
                else if (strcmp(action, "SPIN_RIGHT") == 0) set_motor_speeds(-MOTOR_MAX_DUTY, MOTOR_MAX_DUTY);
                else if (strcmp(action, "STOP") == 0) drive_stop();
                else if (strcmp(action, "ARM_UP") == 0) apply_arm_sync(arm_pct + 5.0f);
                else if (strcmp(action, "ARM_DOWN") == 0) apply_arm_sync(arm_pct - 5.0f);
                else if (strcmp(action, "CUP_UP") == 0) apply_cup(cup_angle + 5.0f);
                else if (strcmp(action, "CUP_DOWN") == 0) apply_cup(cup_angle - 5.0f);
                else if (strcmp(action, "CLAW_OPEN") == 0) apply_claw(claw_angle + 5.0f);
                else if (strcmp(action, "CLAW_CLOSE") == 0) apply_claw(claw_angle - 5.0f);
            }
        }
    }
    httpd_resp_send(req, "OK", 2);
    return ESP_OK;
}

static void start_control_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };
    httpd_uri_t cmd_uri = {
        .uri       = "/cmd",
        .method    = HTTP_GET,
        .handler   = cmd_handler,
        .user_ctx  = NULL
    };

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &cmd_uri);
        ESP_LOGI(TAG, "Server Web pornit cu succes!");
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGW(TAG, "Deconectat de la telefon. Reconectare...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "==================================================");
        ESP_LOGI(TAG, "CONECTAT LA TELEFON!");
        ESP_LOGI(TAG, "Deschide in Chrome pe telefon: http://" IPSTR "/", IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "==================================================");
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Automatically request an IP from the phone via DHCP
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = HOTSPOT_SSID,
            .password = HOTSPOT_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_wifi_set_max_tx_power(78);
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_LOGI(TAG, "Se conecteaza la hotspot-ul '%s'...", HOTSPOT_SSID);
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

void safety_watchdog_task(void *pvParameters) {
    while (1) {
        if (motors_active && (esp_timer_get_time() - last_drive_cmd_time > DRIVE_TIMEOUT_US)) {
            drive_stop();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Servos snap into position right here
    init_actuators();
    
    // Connect to phone hotspot
    wifi_init_sta();

    // Start local webserver on port 80
    start_control_webserver();

    xTaskCreatePinnedToCore(safety_watchdog_task, "watchdog", 2048, NULL, 10, NULL, 1);
}