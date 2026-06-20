#pragma once
#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "sensor.h"

// ─────────────────────────────────────────
//  GLOBAL PID STATE
// ─────────────────────────────────────────
extern int g_error;
extern int g_last_error;
extern int g_pv_out;
extern int g_out_p;
extern int g_out_d;
extern int g_LOUT;
extern int g_ROUT;

const int PID_SP = 0;

// ─────────────────────────────────────────
//  KALKULASI PID
// ─────────────────────────────────────────
void calc_pid(uint8_t kp, uint8_t kd) {
    static unsigned long lastTime = 0;
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;
    lastTime = now;

    g_out_p      = g_error * (int)kp;
    g_out_d      = (int)((g_error - g_last_error) * (int)kd / dt);
    g_last_error = g_error;
}

// ─────────────────────────────────────────
//  RESET PID
// ─────────────────────────────────────────
void clear_pid() {
    g_error      = 0;
    g_last_error = 0;
    g_pv_out     = 0;
    g_out_p      = 0;
    g_out_d      = 0;
    g_LOUT       = 0;
    g_ROUT       = 0;
}

// ─────────────────────────────────────────
//  MODE NORMAL — FOLLOWING
// ─────────────────────────────────────────
void mode_normal(uint8_t kecepatan, uint8_t maxpwm, uint8_t kp, uint8_t kd, LineColor line) {
    
    if (line == LINE_PUTIH) {
        g_line_param = 2;  // invert
    } else if (line == LINE_HITAM) {
        g_line_param = 1;  // normal
    } else {
        g_line_param = 0;  // auto
    }
    g_mode_flag = 1;  // selalu thin untuk mode normal
    scan_sensor();
    if (g_sensor_out != 0) g_pv_out = input_error(g_sensor_out);
    g_error = -g_pv_out;

    calc_pid(kp, kd);

    g_LOUT = constrain((int)kecepatan + g_out_p + g_out_d, -120, 255);
    g_ROUT = constrain((int)kecepatan - g_out_p - g_out_d, -120, 255);
    set_motors(g_LOUT, g_ROUT, maxpwm);
}