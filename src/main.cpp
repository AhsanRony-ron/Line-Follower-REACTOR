// Created by Ron with salving 4 AI

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

#include "config.h"
#include "types.h"
#include "hardware.h"
#include "motor.h"
#include "sensor.h"
#include "pid.h"
#include "eeprom.h"
#include "display.h"
#include "menu.h"
#include "mode_counter.h"

// ─────────────────────────────────────────
//  GLOBAL STATE
// ─────────────────────────────────────────


U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE);

GlobalConfig    g_config;
CounterParam    g_counter[COUNTER_MAX];     // counter 0-99
CheckpointParam g_checkpoint[CP_MAX];       // max 10 CP

volatile int  g_timer   = 0;    // cacahan timer (naik setiap periode×1ms)
volatile int  g_cacah   = 0;    // sub-cacah untuk pembagi periode
volatile uint16_t g_periodo_isr = 3;  // volatile copy of g_config.periode untuk ISR (thread-safe)
uint8_t       g_counter_idx = 0;

// Sensor
uint8_t  g_sensor_raw[SENSOR_COUNT];
uint8_t  g_sensor_thresh[SENSOR_COUNT];
uint16_t g_sensor_out     = 0;
uint8_t  g_invers         = 0;  // flag auto-recovery keluar jalur
uint8_t  g_sensor_state[SENSOR_COUNT] = {0};

// PID
int g_error      = 0;
int g_last_error = 0;
int g_pv         = 0;
int g_pv_out     = 0;
int g_out_p      = 0;
int g_out_d      = 0;
int g_LOUT       = 0;
int g_ROUT       = 0;

volatile unsigned long last_timer_ms = 0;
volatile bool tanda_mode_counter = false;

// ─────────────────────────────────────────
//  INTERRUPT — Timer2 setiap 10ms
// ─────────────────────────────────────────

void timer_isr() {
    g_cacah++;
    if (g_cacah >= (int)g_config.periode) {
        g_timer++;
        g_cacah = 0;
    }
}

void timer_init1() {
    // Software timer using millis() - called from loop()
    last_timer_ms = millis();
}

void timer_update() {
    unsigned long now = millis();

    if (now - last_timer_ms >= g_config.periode) {
    last_timer_ms = now;
    timer_isr();
    }
}


// ─────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────

void setup() {
    display_init();
    display_msg("Loading...");
    hardware_init();
    Wire.setClock(400000);
    eeprom_init();
    timer_init1();
}

// ─────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────

void loop() {
    timer_update();
    // layar standby — pilih CP, tunggu start atau masuk menu
    uint8_t cp_sel = screen_standby();

    // jalankan robot sesuai mode
    if (g_config.mode == MODE_NORMAL) {
        // mode normal: following PID biasa sampai tombol ditekan
        unsigned long t_start = millis();
        clear_pid();
        encoderKiriReset(); 
        encoderKananReset();
        while (!sw_save() && !sw_x()) {
            mode_normal(
                g_config.speed_mode,
                DEFAULT_MAX_PWM,
                g_config.kp,
                g_config.kd,
                g_config.line
            );
            // display_running(
            //     0, 0, g_config.speed_mode,
            //     g_config.kp, millis() - t_start
            // );
        }
        motor_stop();

    } else {
        // mode counter
        encoderKiriReset();
        encoderKananReset();
        mode_counter(cp_sel);
    }
}