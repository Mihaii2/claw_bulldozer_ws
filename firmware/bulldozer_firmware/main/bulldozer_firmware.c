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

#define TAG "BULLDOZER_WS"

#define HOTSPOT_SSID        "Project"
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
    .arm_l_down = 136.0f,
    .arm_l_up   = 44.0f,
    .arm_r_down = 36.0f,
    .arm_r_up   = 124.0f
};

typedef struct {
    float target;
    float current;
    float max_step;
} servo_channel_state_t;

static servo_channel_state_t servo_cam   = { .target = 180.0f, .current = 180.0f, .max_step = 8.0f };
static servo_channel_state_t servo_arm_l = { .target = 99.2f,  .current = 99.2f,  .max_step = 5.0f };
static servo_channel_state_t servo_arm_r = { .target = 71.2f,  .current = 71.2f,  .max_step = 5.0f };
static servo_channel_state_t servo_cup   = { .target = 90.0f,  .current = 90.0f,  .max_step = 6.0f };
static servo_channel_state_t servo_claw  = { .target = 90.0f,  .current = 90.0f,  .max_step = 8.0f };

static portMUX_TYPE angle_mux = portMUX_INITIALIZER_UNLOCKED;

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

    // Left motor
    if (left_pwm >= 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN1, left_pwm);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN2, 0);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN1, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_L_IN2, -left_pwm);
    }

    // Right motor
    if (right_pwm >= 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_MOT_R_IN3, 0);
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

static void servo_smoothing_task(void *pvParameters) {
    while (1) {
        taskENTER_CRITICAL(&angle_mux);

        if (fabsf(servo_cam.current - servo_cam.target) > 0.1f) {
            float delta = servo_cam.target - servo_cam.current;
            if (fabsf(delta) <= servo_cam.max_step) servo_cam.current = servo_cam.target;
            else servo_cam.current += (delta > 0) ? servo_cam.max_step : -servo_cam.max_step;
            apply_raw_servo(CH_SERVO_CAM, servo_cam.current);
        }

        if (fabsf(servo_arm_l.current - servo_arm_l.target) > 0.1f) {
            float delta = servo_arm_l.target - servo_arm_l.current;
            if (fabsf(delta) <= servo_arm_l.max_step) servo_arm_l.current = servo_arm_l.target;
            else servo_arm_l.current += (delta > 0) ? servo_arm_l.max_step : -servo_arm_l.max_step;
            apply_raw_servo(CH_SERVO_ARM_L, servo_arm_l.current);
        }

        if (fabsf(servo_arm_r.current - servo_arm_r.target) > 0.1f) {
            float delta = servo_arm_r.target - servo_arm_r.current;
            if (fabsf(delta) <= servo_arm_r.max_step) servo_arm_r.current = servo_arm_r.target;
            else servo_arm_r.current += (delta > 0) ? servo_arm_r.max_step : -servo_arm_r.max_step;
            apply_raw_servo(CH_SERVO_ARM_R, servo_arm_r.current);
        }

        if (fabsf(servo_cup.current - servo_cup.target) > 0.1f) {
            float delta = servo_cup.target - servo_cup.current;
            if (fabsf(delta) <= servo_cup.max_step) servo_cup.current = servo_cup.target;
            else servo_cup.current += (delta > 0) ? servo_cup.max_step : -servo_cup.max_step;
            apply_raw_servo(CH_SERVO_CUP, servo_cup.current);
        }

        if (fabsf(servo_claw.current - servo_claw.target) > 0.1f) {
            float delta = servo_claw.target - servo_claw.current;
            if (fabsf(delta) <= servo_claw.max_step) servo_claw.current = servo_claw.target;
            else servo_claw.current += (delta > 0) ? servo_claw.max_step : -servo_claw.max_step;
            apply_raw_servo(CH_SERVO_CLAW, servo_claw.current);
        }

        taskEXIT_CRITICAL(&angle_mux);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
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

static const char INDEX_HTML[] = 
"<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'>"
"<title>Bulldozer WebSocket Dashboard</title><style>"
"body{background:#121212;color:#eee;font-family:sans-serif;margin:0;padding:12px;}"
"h2{margin:0 0 12px 0;color:#f39c12;text-align:center;font-size:20px;user-select:none;}"
".layout{display:flex;flex-direction:row;gap:14px;max-width:1150px;margin:0 auto;align-items:flex-start;}"
".col-left{flex:1.1;display:flex;flex-direction:column;gap:10px;min-width:320px;}"
".col-right{flex:1.2;display:flex;flex-direction:column;gap:10px;min-width:340px;}"
"@media(max-width:768px){.layout{flex-direction:column;}}"
".card{background:#1e1e1e;border:1px solid #333;border-radius:10px;padding:12px 14px;box-shadow:0 2px 8px rgba(0,0,0,0.5);}"
".card-header{display:flex;justify-content:space-between;align-items:center;font-weight:bold;font-size:14px;user-select:none;}"
".angle-badge{background:#f39c12;color:#111;padding:2px 8px;border-radius:6px;font-family:monospace;font-size:15px;font-weight:bold;}"
"input[type=range]{width:100%;height:8px;accent-color:#f39c12;cursor:pointer;margin:8px 0;}"
".sub-slider{display:flex;align-items:center;gap:8px;font-size:12px;color:#aaa;margin-top:6px;}"
".sub-slider span{min-width:70px;}"
".drive-box{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;max-width:280px;margin:10px auto 0 auto;user-select:none;}"
"button{background:#333;color:#fff;border:1px solid #555;padding:12px;font-size:16px;border-radius:8px;font-weight:bold;cursor:pointer;touch-action:none;user-select:none;}"
"button:active{background:#f39c12;color:#000;}"
".btn-stop{background:#c0392b;border-color:#e74c3c;}"
".btn-diag{background:#252525;color:#f39c12;font-size:14px;}"
".calib-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:8px;font-size:11px;color:#aaa;}"
".calib-box{background:#2a2a2a;padding:6px;border-radius:6px;}"
".calib-box input{width:100%;box-sizing:border-box;background:#111;border:1px solid #555;color:#f39c12;padding:4px;font-size:13px;border-radius:4px;text-align:center;}"
".stat-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin:8px 0;font-size:11px;text-align:center;user-select:none;}"
".stat-box{background:#181818;border:1px solid #333;border-radius:6px;padding:6px 2px;}"
".stat-val{font-size:14px;font-weight:bold;font-family:monospace;color:#2ecc71;margin-top:2px;}"
".stat-warn{color:#e74c3c;}"
"#log_box{background:#0a0a0a;border:1px solid #333;border-radius:6px;font-family:monospace;font-size:11px;height:420px;overflow-y:scroll;padding:8px;color:#bbb;margin-top:8px;user-select:text!important;cursor:text;resize:vertical;white-space:pre;line-height:1.4;}"
".toolbar{display:flex;gap:8px;margin-top:6px;}"
".btn-small{padding:5px 10px;font-size:12px;background:#2c3e50;border:1px solid #34495e;border-radius:4px;color:#ecf0f1;cursor:pointer;}"
".btn-small:active{background:#1abc9c;}"
"</style>"
"<script>"
"window.rawLogHistory = [];"
"window.allTimeMax = 0;"
"window.totalRtt = 0; window.totalCount = 0;"
"window.recentSamples = [];"
"window.activeCmd = 'STOP';"
"window.lastSentCmd = 'STOP';"
"window.touchCmd = null;"
"window.pressedKeys = new Set();"
"window.ws = null;"
"window.lastPingTime = 0;"

"function initWS() {"
"  let loc = window.location;"
"  let uri = 'ws://' + loc.host + '/ws';"
"  window.ws = new WebSocket(uri);"
"  window.ws.onopen = function() {"
"    document.getElementById('status_dot').innerText = '● WS CONNECTED';"
"    document.getElementById('status_dot').style.color = '#2ecc71';"
"  };"
"  window.ws.onmessage = function(e) {"
"    let now = performance.now();"
"    let rtt = now - window.lastPingTime;"
"    window.recordLatency(rtt, 'WS: ' + e.data);"
"  };"
"  window.ws.onclose = function() {"
"    document.getElementById('status_dot').innerText = '○ WS RECONNECTING';"
"    document.getElementById('status_dot').style.color = '#e74c3c';"
"    setTimeout(initWS, 1500);"
"  };"
"}"
"window.addEventListener('load', initWS);"

"window.recordLatency = function(ms, label) {"
"  let now = performance.now();"
"  window.totalRtt += ms;"
"  window.totalCount++;"
"  if (ms > window.allTimeMax && ms < 3000) window.allTimeMax = ms;"
"  window.recentSamples.push({ time: now, rtt: ms });"
"  window.recentSamples = window.recentSamples.filter(x => x.time >= (now - 10000));"
"  let max10 = 0;"
"  for (let s of window.recentSamples) { if (s.rtt > max10 && s.rtt < 3000) max10 = s.rtt; }"
"  let pElem = document.getElementById('stat_ping');"
"  if(pElem) { pElem.innerText = Math.round(ms) + ' ms'; pElem.className = 'stat-val' + (ms > 50 ? ' stat-warn' : ''); }"
"  let avgElem = document.getElementById('stat_avg');"
"  if(avgElem) avgElem.innerText = Math.round(window.totalRtt / window.totalCount) + ' ms';"
"  let m10Elem = document.getElementById('stat_max10');"
"  if(m10Elem) { m10Elem.innerText = Math.round(max10) + ' ms'; m10Elem.className = 'stat-val' + (max10 > 60 ? ' stat-warn' : ''); }"
"  let mAllElem = document.getElementById('stat_max_all');"
"  if(mAllElem) { mAllElem.innerText = Math.round(window.allTimeMax) + ' ms'; mAllElem.className = 'stat-val' + (window.allTimeMax > 70 ? ' stat-warn' : ''); }"
"  let d = new Date();"
"  let ts = d.toTimeString().split(' ')[0] + '.' + ('00' + d.getMilliseconds()).slice(-3);"
"  let textLine = '[' + ts + '] ' + label.padEnd(14) + ' | RTT: ' + ms.toFixed(1) + 'ms\\n';"
"  window.rawLogHistory.push(textLine);"
"  if (window.rawLogHistory.length > 600) window.rawLogHistory.shift();"
"  let box = document.getElementById('log_box');"
"  if(box) box.textContent = textLine + box.textContent;"
"};"

"window.copyLogs = function() {"
"  let fullText = '=== BULLDOZER TELEMETRY LOGS ===\\n' +"
"    'Stats: Avg=' + Math.round(window.totalRtt/window.totalCount) + 'ms | AllTimeMax=' + Math.round(window.allTimeMax) + 'ms\\n\\n' +"
"    window.rawLogHistory.slice().reverse().join('');"
"  let ta = document.createElement('textarea');"
"  ta.value = fullText;"
"  ta.style.position = 'fixed'; ta.style.left = '-9999px';"
"  document.body.appendChild(ta);"
"  ta.focus(); ta.select();"
"  try { document.execCommand('copy'); }"
"  catch(err) {}"
"  document.body.removeChild(ta);"
"  let alert = document.getElementById('copy_alert');"
"  if(alert) { alert.style.display = 'inline'; setTimeout(()=>{ alert.style.display = 'none'; }, 2000); }"
"};"

"window.clearLogs = function() {"
"  window.rawLogHistory = [];"
"  let box = document.getElementById('log_box');"
"  if(box) box.textContent = '';"
"  window.totalRtt = 0; window.totalCount = 0; window.allTimeMax = 0;"
"};"

"window.sendAngleDirect = function(joint, val){"
"  let badge = document.getElementById('val_' + joint);"
"  if(badge) badge.innerText = Number(val).toFixed(1) + '°';"
"  if(window.ws && window.ws.readyState === WebSocket.OPEN){"
"    window.ws.send('S:' + joint + ':' + val);"
"  }"
"};"

"window.sendArmSyncDirect = function(percent){"
"  let badge = document.getElementById('val_arm_sync');"
"  if(badge) badge.innerText = percent + '%';"
"  let ld = parseFloat(document.getElementById('cal_l_down').value);"
"  let lu = parseFloat(document.getElementById('cal_l_up').value);"
"  let rd = parseFloat(document.getElementById('cal_r_down').value);"
"  let ru = parseFloat(document.getElementById('cal_r_up').value);"
"  let p = percent / 100.0;"
"  let l = ld - p * (ld - lu);"
"  let r = rd + p * (ru - rd);"
"  document.getElementById('sl_arm_l').value = l.toFixed(0);"
"  document.getElementById('sl_arm_r').value = r.toFixed(0);"
"  document.getElementById('val_arm_l').innerText = l.toFixed(1) + '°';"
"  document.getElementById('val_arm_r').innerText = r.toFixed(1) + '°';"
"  if(window.ws && window.ws.readyState === WebSocket.OPEN){"
"    window.ws.send('S:arms_sync:' + percent);"
"  }"
"};"

"window.updateCalib = function(){"
"  let ld = document.getElementById('cal_l_down').value;"
"  let lu = document.getElementById('cal_l_up').value;"
"  let rd = document.getElementById('cal_r_down').value;"
"  let ru = document.getElementById('cal_r_up').value;"
"  fetch('/calib?ld=' + ld + '&lu=' + lu + '&rd=' + rd + '&ru=' + ru).catch(()=>{});"
"  window.sendArmSyncDirect(document.getElementById('sl_arm_sync').value);"
"};"

"window.evaluateKeys = function() {"
"  let w = window.pressedKeys.has('KeyW') || window.pressedKeys.has('ArrowUp');"
"  let s = window.pressedKeys.has('KeyS') || window.pressedKeys.has('ArrowDown');"
"  let a = window.pressedKeys.has('KeyA') || window.pressedKeys.has('ArrowLeft');"
"  let d = window.pressedKeys.has('KeyD') || window.pressedKeys.has('ArrowRight');"
"  if (w && d) return 'FWD_RIGHT';"
"  if (w && a) return 'FWD_LEFT';"
"  if (s && d) return 'REV_RIGHT';"
"  if (s && a) return 'REV_LEFT';"
"  if (w) return 'FORWARD';"
"  if (s) return 'REVERSE';"
"  if (a) return 'SPIN_LEFT';"
"  if (d) return 'SPIN_RIGHT';"
"  return null;"
"};"

"window.sendDriveCommand = function() {"
"  let target = window.touchCmd ? window.touchCmd : window.evaluateKeys();"
"  if (!target) target = 'STOP';"
"  window.activeCmd = target;"
"  if (window.activeCmd === 'STOP' && window.lastSentCmd === 'STOP') return;"
"  window.lastSentCmd = window.activeCmd;"
"  if(window.ws && window.ws.readyState === WebSocket.OPEN){"
"    window.lastPingTime = performance.now();"
"    window.ws.send('C:' + window.activeCmd);"
"  }"
"};"

"window.setTouchCmd = function(cmd){ window.touchCmd = cmd; window.sendDriveCommand(); };"
"window.clearTouchCmd = function(){ window.touchCmd = null; window.sendDriveCommand(); };"
"window.emergencyStop = function(){ window.touchCmd = null; window.pressedKeys.clear(); window.activeCmd = 'STOP'; window.lastSentCmd = 'STOP'; if(window.ws) window.ws.send('C:STOP'); };"

"setInterval(()=>{"
"  if(window.activeCmd !== 'STOP') {"
"    window.sendDriveCommand();"
"  }"
"}, 60);"

"window.addEventListener('keydown', (e)=>{ if(e.repeat) return; if(['KeyW','KeyS','KeyA','KeyD','ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.code)){ window.pressedKeys.add(e.code); window.sendDriveCommand(); } });"
"window.addEventListener('keyup', (e)=>{ if(window.pressedKeys.has(e.code)){ window.pressedKeys.delete(e.code); window.sendDriveCommand(); } });"
"window.addEventListener('blur', ()=>{ window.pressedKeys.clear(); window.sendDriveCommand(); });"
"</script>"
"</head><body>"
"<h2>🚜 Bulldozer High-Speed WebSocket Control & Profiler</h2>"
"<div class='layout'>"

"<!-- LEFT COLUMN: ACTUATORS & DRIVE -->"
"<div class='col-left'>"
"  <div class='card'>"
"    <div class='card-header'><span>📱 Phone / Cam Pitch (GPIO 4)</span><span id='val_cam' class='angle-badge'>180.0°</span></div>"
"    <input type='range' id='sl_cam' min='0' max='180' step='1' value='180' oninput='sendAngleDirect(\"cam\", this.value)'>"
"  </div>"

"  <div class='card'>"
"    <div class='card-header'><span>🏗️️ Dual Arm Height (Synced)</span><span id='val_arm_sync' class='angle-badge'>40%</span></div>"
"    <input type='range' id='sl_arm_sync' min='0' max='100' step='1' value='40' oninput='sendArmSyncDirect(this.value)'>"
"    <div class='calib-grid'>"
"      <div class='calib-box'>"
"        <b>Left Arm (Down / Up)</b>"
"        <input type='number' id='cal_l_down' value='136' onchange='updateCalib()'> down"
"        <input type='number' id='cal_l_up' value='44' onchange='updateCalib()'> up"
"      </div>"
"      <div class='calib-box'>"
"        <b>Right Arm (Down / Up)</b>"
"        <input type='number' id='cal_r_down' value='36' onchange='updateCalib()'> down"
"        <input type='number' id='cal_r_up' value='124' onchange='updateCalib()'> up"
"      </div>"
"    </div>"
"  </div>"

"  <div class='card'>"
"    <div class='card-header'><span>Individual Arm Override</span></div>"
"    <div class='sub-slider'>"
"      <span>Left (G18)</span>"
"      <input type='range' id='sl_arm_l' min='0' max='180' step='1' value='99' oninput='sendAngleDirect(\"arm_l\", this.value)'>"
"      <span id='val_arm_l' style='min-width:40px;'>99.2°</span>"
"    </div>"
"    <div class='sub-slider'>"
"      <span>Right (G5)</span>"
"      <input type='range' id='sl_arm_r' min='0' max='180' step='1' value='71' oninput='sendAngleDirect(\"arm_r\", this.value)'>"
"      <span id='val_arm_r' style='min-width:40px;'>71.2°</span>"
"    </div>"
"  </div>"

"  <div class='card'>"
"    <div class='card-header'><span>🪣 Wrist / Cup Pitch (GPIO 17)</span><span id='val_cup' class='angle-badge'>90.0°</span></div>"
"    <input type='range' id='sl_cup' min='0' max='180' step='1' value='90' oninput='sendAngleDirect(\"cup\", this.value)'>"
"  </div>"

"  <div class='card'>"
"    <div class='card-header'><span>🤏 Claw Gripper (GPIO 16)</span><span id='val_claw' class='angle-badge'>90.0°</span></div>"
"    <input type='range' id='sl_claw' min='0' max='180' step='1' value='90' oninput='sendAngleDirect(\"claw\", this.value)'>"
"  </div>"

"  <div class='card'>"
"    <div class='card-header'><span>🚜 Drive Chassis (Instant WebSocket Mode)</span></div>"
"    <div class='drive-box'>"
"      <button class='btn-diag' onpointerdown=\"setTouchCmd('FWD_LEFT')\" onpointerup=\"clearTouchCmd()\">↖</button>"
"      <button onpointerdown=\"setTouchCmd('FORWARD')\" onpointerup=\"clearTouchCmd()\">▲</button>"
"      <button class='btn-diag' onpointerdown=\"setTouchCmd('FWD_RIGHT')\" onpointerup=\"clearTouchCmd()\">↗</button>"

"      <button onpointerdown=\"setTouchCmd('SPIN_LEFT')\" onpointerup=\"clearTouchCmd()\">◄</button>"
"      <button class='btn-stop' onclick=\"emergencyStop()\">■</button>"
"      <button onpointerdown=\"setTouchCmd('SPIN_RIGHT')\" onpointerup=\"clearTouchCmd()\">►</button>"

"      <button class='btn-diag' onpointerdown=\"setTouchCmd('REV_LEFT')\" onpointerup=\"clearTouchCmd()\">↙</button>"
"      <button onpointerdown=\"setTouchCmd('REVERSE')\" onpointerup=\"clearTouchCmd()\">▼</button>"
"      <button class='btn-diag' onpointerdown=\"setTouchCmd('REV_RIGHT')\" onpointerup=\"clearTouchCmd()\">↘</button>"
"    </div>"
"  </div>"
"</div>"

"<!-- RIGHT COLUMN: LATENCY STATS & LOGS -->"
"<div class='col-right'>"
"  <div class='card'>"
"    <div class='card-header'><span>📊 WebSocket Real-Time Latency</span><span id='status_dot' style='color:#e74c3c;'>○ CONNECTING</span></div>"
"    <div class='stat-grid'>"
"      <div class='stat-box'><div>Current Ping</div><div id='stat_ping' class='stat-val'>-- ms</div></div>"
"      <div class='stat-box'><div>Avg (All Time)</div><div id='stat_avg' class='stat-val'>-- ms</div></div>"
"      <div class='stat-box'><div>Max (10s)</div><div id='stat_max10' class='stat-val'>-- ms</div></div>"
"      <div class='stat-box'><div>All-Time Max</div><div id='stat_max_all' class='stat-val'>-- ms</div></div>"
"    </div>"
"    <div class='toolbar'>"
"      <button class='btn-small' onclick='copyLogs()'>📋 Copy All Logs</button>"
"      <button class='btn-small' onclick='clearLogs()'>🗑️ Clear</button>"
"      <span id='copy_alert' style='font-size:11px;color:#2ecc71;display:none;align-self:center;'>Copied!</span>"
"    </div>"
"    <div id='log_box'></div>"
"  </div>"
"</div>"

"</div></body></html>";

static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t favicon_handler(httpd_req_t *req) {
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static void handle_drive_action(const char *action) {
    if (strcmp(action, "FORWARD") == 0) set_motor_speeds(MOTOR_MAX_DUTY, MOTOR_MAX_DUTY);
    else if (strcmp(action, "REVERSE") == 0) set_motor_speeds(-MOTOR_MAX_DUTY, -MOTOR_MAX_DUTY);
    else if (strcmp(action, "SPIN_LEFT") == 0) set_motor_speeds(MOTOR_MAX_DUTY, -MOTOR_MAX_DUTY);
    else if (strcmp(action, "SPIN_RIGHT") == 0) set_motor_speeds(-MOTOR_MAX_DUTY, MOTOR_MAX_DUTY);
    else if (strcmp(action, "FWD_RIGHT") == 0) set_motor_speeds(0, MOTOR_MAX_DUTY);
    else if (strcmp(action, "FWD_LEFT") == 0) set_motor_speeds(MOTOR_MAX_DUTY, 0);
    else if (strcmp(action, "REV_RIGHT") == 0) set_motor_speeds(0, -MOTOR_MAX_DUTY);
    else if (strcmp(action, "REV_LEFT") == 0) set_motor_speeds(-MOTOR_MAX_DUTY, 0);
    else if (strcmp(action, "STOP") == 0) drive_stop();
}

static void handle_servo_action(const char *joint, float val) {
    taskENTER_CRITICAL(&angle_mux);
    if (strcmp(joint, "cam") == 0) {
        servo_cam.target = val;
    } else if (strcmp(joint, "arms_sync") == 0) {
        float p = val / 100.0f;
        if (p < 0.0f) p = 0.0f;
        if (p > 1.0f) p = 1.0f;
        servo_arm_l.target = arm_cal.arm_l_down - p * (arm_cal.arm_l_down - arm_cal.arm_l_up);
        servo_arm_r.target = arm_cal.arm_r_down + p * (arm_cal.arm_r_up - arm_cal.arm_r_down);
    } else if (strcmp(joint, "arm_l") == 0) {
        servo_arm_l.target = val;
    } else if (strcmp(joint, "arm_r") == 0) {
        servo_arm_r.target = val;
    } else if (strcmp(joint, "cup") == 0) {
        servo_cup.target = val;
    } else if (strcmp(joint, "claw") == 0) {
        servo_claw.target = val;
    }
    taskEXIT_CRITICAL(&angle_mux);
}

static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket handshake established");
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t buf[64];
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = buf;
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, sizeof(buf) - 1);
    if (ret != ESP_OK) return ret;

    buf[ws_pkt.len] = '\0';
    char *msg = (char *)buf;

    // 1. Drive Commands: "C:<CMD>" (e.g., "C:FORWARD")
    if (msg[0] == 'C' && msg[1] == ':') {
        char *action = msg + 2;
        handle_drive_action(action);

        // Echo back action immediately for round-trip latency benchmarking
        httpd_ws_frame_t resp_pkt = {
            .payload = (uint8_t *)action,
            .len = strlen(action),
            .type = HTTPD_WS_TYPE_TEXT
        };
        httpd_ws_send_frame(req, &resp_pkt);
    } 
    // 2. Servo Angle Commands: "S:<JOINT>:<VAL>" (e.g., "S:cam:120.0" or "S:arms_sync:40")
    else if (msg[0] == 'S' && msg[1] == ':') {
        char *joint = strtok(msg + 2, ":");
        char *val_str = strtok(NULL, ":");
        if (joint && val_str) {
            handle_servo_action(joint, atof(val_str));
        }
    }
    // 3. Calibration Trim Limits: "K:<ld>:<lu>:<rd>:<ru>"
    else if (msg[0] == 'K' && msg[1] == ':') {
        char *token = strtok(msg + 2, ":");
        taskENTER_CRITICAL(&angle_mux);
        if (token) arm_cal.arm_l_down = atof(token);
        token = strtok(NULL, ":");
        if (token) arm_cal.arm_l_up = atof(token);
        token = strtok(NULL, ":");
        if (token) arm_cal.arm_r_down = atof(token);
        token = strtok(NULL, ":");
        if (token) arm_cal.arm_r_up = atof(token);
        taskEXIT_CRITICAL(&angle_mux);

        ESP_LOGI(TAG, "WS CALIB SYNC: L(%.1f, %.1f) R(%.1f, %.1f)", 
                 arm_cal.arm_l_down, arm_cal.arm_l_up, arm_cal.arm_r_down, arm_cal.arm_r_up);
    }

    return ESP_OK;
}

static esp_err_t calib_handler(httpd_req_t *req) {
    char buf[128];
    size_t buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1 && buf_len <= sizeof(buf)) {
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            char v[16];
            taskENTER_CRITICAL(&angle_mux);
            if (httpd_query_key_value(buf, "ld", v, sizeof(v)) == ESP_OK) arm_cal.arm_l_down = atof(v);
            if (httpd_query_key_value(buf, "lu", v, sizeof(v)) == ESP_OK) arm_cal.arm_l_up = atof(v);
            if (httpd_query_key_value(buf, "rd", v, sizeof(v)) == ESP_OK) arm_cal.arm_r_down = atof(v);
            if (httpd_query_key_value(buf, "ru", v, sizeof(v)) == ESP_OK) arm_cal.arm_r_up = atof(v);
            taskEXIT_CRITICAL(&angle_mux);
        }
    }
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_status(req, "200 OK");
    return httpd_resp_send(req, "OK", 2);
}

static void start_control_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_open_sockets = 7;
    config.lru_purge_enable = true;

    httpd_uri_t index_uri   = { .uri = "/",            .method = HTTP_GET, .handler = index_handler };
    httpd_uri_t favicon_uri = { .uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_handler };
    httpd_uri_t calib_uri   = { .uri = "/calib",        .method = HTTP_GET, .handler = calib_handler };
    httpd_uri_t ws_uri      = {
        .uri        = "/ws",
        .method     = HTTP_GET,
        .handler    = ws_handler,
        .user_ctx   = NULL,
        .is_websocket = true
    };

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &favicon_uri);
        httpd_register_uri_handler(server, &calib_uri);
        httpd_register_uri_handler(server, &ws_uri);
        ESP_LOGI(TAG, "High-Speed WebSocket server running on port 80");
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "==================================================");
        ESP_LOGI(TAG, "WEBSOCKET TELEOP DASHBOARD READY");
        ESP_LOGI(TAG, "URL: http://" IPSTR "/", IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "==================================================");
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
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

    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

void safety_watchdog_task(void *pvParameters) {
    while (1) {
        if (motors_active && (esp_timer_get_time() - last_drive_cmd_time > DRIVE_TIMEOUT_US)) {
            drive_stop();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
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
    start_control_webserver();

    xTaskCreatePinnedToCore(servo_smoothing_task, "servo_slew", 2048, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(safety_watchdog_task, "watchdog", 2048, NULL, 10, NULL, 1);
}