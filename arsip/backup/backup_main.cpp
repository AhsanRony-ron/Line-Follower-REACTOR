#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

Adafruit_SSD1306 lcd(128, 64, &Wire, -1);

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
// uint8_t  g_sensor_state[SENSOR_COUNT] = {0};

// PID
int g_error      = 0;
int g_last_error = 0;
int g_pv         = 0;
int g_pv_out     = 0;
int g_out_p      = 0;
int g_out_d      = 0;
int g_LOUT       = 0;
int g_ROUT       = 0;

void timer_isr() {

    g_cacah++;
    if (g_cacah >= (int)g_periodo_isr) {
        g_timer++;
        g_cacah = 0;
    }
}

// ─────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────

void setup() {
    analogWriteResolution(8);  // pastikan 0-255
    Wire.begin();
    Wire.setClock(400000);
    display_init();
    display_msg("Loading...");
    hardware_init();
    eeprom_init();
    g_periodo_isr = g_config.periode;  // Sync volatile copy setelah load dari EEPROM
}

// ─────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────

void loop() {
    // layar standby — pilih CP, tunggu start atau masuk menu
    uint8_t cp_sel = screen_standby();

    // jalankan robot sesuai mode
    if (g_config.mode == MODE_NORMAL) {
        // mode normal: following PID biasa sampai tombol ditekan
        unsigned long t_start = millis();
        clear_pid();
        while (!sw_save() && !sw_x()) {
            mode_normal(
                g_config.speed_mode,
                DEFAULT_MAX_PWM,
                g_config.kp,
                g_config.kd,
                g_config.line
            );
            display_running(
                0, 0, g_config.speed_mode,
                g_config.kp, millis() - t_start
            );
        }
        motor_stop();

    } else {
        // mode counter
        mode_counter(cp_sel);
    }

    // ❌ REMOVED: Automatic eeprom_save_all() setelah setiap run
    // Alasan: EEPROM endurance terbatas (~1M write cycle)
    // Kalau save otomatis setiap run, EEPROM akan cepat aus
    // Sekarang hanya save saat:
    //   1. User explicit save dari menu (Menu1/Menu2)
    //   2. User swap memory slot
    //   3. User delete atau reset dari menu
    // Untuk safety, user harus tekan SAVE di standby untuk menyimpan data ke EEPROM
}