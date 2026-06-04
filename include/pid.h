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
    g_out_p      = g_error * kp;
    g_out_d      = (g_error - g_last_error) * kd;
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
    decode_zone(line);
    scan_sensor();
    if (g_sensor_out != 0) g_pv_out = input_error(g_sensor_out);
    g_error = -g_pv_out;

    calc_pid(kp, kd);

    g_LOUT = constrain((int)kecepatan + g_out_p + g_out_d, -255, 255);
    g_ROUT = constrain((int)kecepatan - g_out_p - g_out_d, -255, 255);

    set_motors(g_LOUT, g_ROUT, maxpwm);
}