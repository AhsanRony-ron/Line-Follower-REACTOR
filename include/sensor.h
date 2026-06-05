#pragma once
#include <Arduino.h>
#include "config.h"
#include "types.h"

// ─────────────────────────────────────────
//  GLOBAL SENSOR STATE
// ─────────────────────────────────────────
extern uint8_t  g_sensor_raw[SENSOR_COUNT]; //analog_val
extern uint8_t  g_sensor_thresh[SENSOR_COUNT]; //sensor_threshold
extern uint8_t  g_sensor_state[SENSOR_COUNT];
extern uint16_t g_sensor_out;   // bitmask 14-bit hasil scan — selalu nilai FINAL
extern int      g_pv_out;       // untuk input_error() default case

// Counter per sensor (berapa kali aktif per scan)
uint8_t sensor_count[14] = {0};

// Counter untuk deteksi noise
uint8_t noise_counter = 0;

// Flag invert mode
uint8_t sensor_inverted = 0;

// Jumlah sensor aktif per scan
uint8_t sensor_active_count = 0;

// Global mode_flag — di-set saat counter advance
uint8_t g_mode_flag  = 1;   // default thin
uint8_t g_line_param = 0;   // default normal

// ─────────────────────────────────────────
//  BACA ADC DARI MULTIPLEXER
//  ch: 0-13, return: nilai 8-bit (0-255)
// ─────────────────────────────────────────

uint8_t read_adc(uint8_t ch) {
    if (ch < 8) {
        digitalWrite(PIN_MUX_A, MUX_A[ch]);
        digitalWrite(PIN_MUX_B, MUX_B[ch]);
        digitalWrite(PIN_MUX_C, MUX_C[ch]);
        delayMicroseconds(20);
        return (uint8_t)(analogRead(PIN_ADC0) >> 4);
    } else {
        uint8_t idx = ch - 8;
        digitalWrite(PIN_MUX_A, MUX_A[idx]);
        digitalWrite(PIN_MUX_B, MUX_B[idx]);
        digitalWrite(PIN_MUX_C, MUX_C[idx]);
        delayMicroseconds(20);
        return (uint8_t)(analogRead(PIN_ADC1) >> 4);
    }
}

// ─────────────────────────────────────────
//  NOISE PATTERN FILTER
// ─────────────────────────────────────────

bool is_noise_pattern(uint16_t bitmask) {
    switch (bitmask) {
        case 0x0fff: case 0x1fff: case 0x23ff: case 0x27ff:
        case 0x2fff: case 0x31ff: case 0x33ff: case 0x37ff:
        case 0x38ff: case 0x39ff: case 0x3bff: case 0x3c7f:
        case 0x3cff: case 0x3dff: case 0x3e3f: case 0x3eff:
        case 0x3f1f: case 0x3f3f: case 0x3f7f: case 0x3f8f:
        case 0x3f9f: case 0x3fbf: case 0x3fc7: case 0x3fcf:
        case 0x3fdf: case 0x3fe3: case 0x3fe7: case 0x3fef:
        case 0x3ff1: case 0x3ff3:
            return true;
        default:
            if (bitmask >= 0x3ff7 && bitmask <= 0x3ff9) return true;
            if (bitmask >= 0x3ffb && bitmask <= 0x3fff) return true;
            return false;
    }
}

// ─────────────────────────────────────────
//  NEIGHBOR VALIDITY CHECK
// ─────────────────────────────────────────

uint16_t get_neighbor(uint16_t bitmask) {
    if (bitmask < 0x2400)  return 0x07ff;
    if (bitmask < 0x3000)  return 0x27ff;
    if (bitmask < 0x3800) {
        if (bitmask <= 0x31ff) return bitmask;
        return 0x33ff;
    }
    if (bitmask < 0x3c80)  return 0x38ff;
    if (bitmask < 0x3e00)  return 0x3cff;
    if (bitmask < 0x3f00)  return 15999;
    if (bitmask < 0x3f90)  return 0x3f1f;
    if (bitmask < 0x3fc0)  return 0x3f9f;
    if (bitmask < 0x3fe0) {
        if (bitmask <= 0x3fc7) return bitmask;
        return 0x3fcf;
    }
    if (bitmask < 0x3ff4)  return 0x3fe3;
    return 0x3ff1;
}

// ─────────────────────────────────────────
//  DECODE ZONE → mode_flag + line_param
// ─────────────────────────────────────────

inline void decode_zone(uint8_t zone_raw) {
    if (zone_raw == 0) {
        g_mode_flag  = 1;
        g_line_param = 0;
    } else if (zone_raw <= 4) {
        g_mode_flag  = zone_raw;
        g_line_param = 0;
    } else if (zone_raw <= 8) {
        g_mode_flag  = zone_raw - 4;
        g_line_param = 1;
    } else {
        g_mode_flag  = zone_raw - 8;
        g_line_param = 2;
    }
}

// ─────────────────────────────────────────
//  SCAN SENSOR BAR (untuk display kalibrasi)
// ─────────────────────────────────────────

void scan_sensor_bar() {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        g_sensor_raw[i] = read_adc(i);
    }
}

// ============================================================
// SCAN SENSOR — read_sensor @ 0x08001560
//
// param_1:
//   0 = normal
//   2 = invert (sensor terbalik, bitmask = 0x3FFF - bitmask)
//
// Return: bitmask 14-bit
//   bit 0  (0x0001) = sensor 0  paling kiri
//   bit 13 (0x2000) = sensor 13 paling kanan
// ============================================================

uint16_t scan_sensor() {
    uint16_t bitmask = 0;

    // Counter per sensor (hanya sensor tertentu punya counter)
    uint8_t cnt[14] = {0};

    // Reset active count
    sensor_active_count = 0;

    // Baca 14 sensor (urutan terbalik: 0xd downto 0)
    for (uint8_t i = 0; i < 14; i++) {
        uint8_t adc = read_adc(i);
        g_sensor_raw[i] = adc;

        // Simpan ke raw buffer (RAM DAT_080017cc + i*4)
        // (disimpan di analog_val[i])

        uint8_t adc8 = adc & 0xFF;

        // Bandingkan dengan threshold (RAM DAT_080017d0 + i)
        if (g_sensor_thresh[i] < adc8) {
            // Set bit di bitmask sesuai urutan binary
            bitmask += (1 << i);
            sensor_active_count++;
            cnt[i] = 1;
        }
    }

    // Simpan bitmask ke out_sensor
    g_sensor_out = bitmask;

    // Simpan counter per sensor ke RAM
    // (sesuai urutan penyimpanan di binary)
    sensor_count[5]  = cnt[5];   // cVar21 → DAT_080017d8
    sensor_count[3]  = cnt[3];   // cVar19 → DAT_080017dc
    sensor_count[2]  = cnt[2];   // cVar8  → DAT_080017e0
    sensor_count[0]  = cnt[0];   // cVar7  → DAT_080017e4
    sensor_count[8]  = cnt[8];   // cVar9  → DAT_080017e8
    sensor_count[9]  = cnt[9];   // cVar10 → DAT_080017ec
    sensor_count[10] = cnt[10];  // cVar11 → DAT_080017f0
    sensor_count[11] = cnt[11];  // cVar12 → DAT_080017f4
    sensor_count[12] = cnt[12];  // cVar13 → DAT_080017f8
    sensor_count[4]  = cnt[4];   // cVar20 → DAT_080017fc
    sensor_count[13] = cnt[13];  // cVar14 → DAT_08001800
    sensor_count[1]  = cnt[1];   // cVar18 → DAT_08001804
    sensor_inverted  = 0;        // DAT_08001808 = 0

    if (g_line_param == 2) {
        // Mode invert: sensor terbalik
        bitmask = 0x3FFF - bitmask;
        g_sensor_out = bitmask;
        for (uint8_t i = 0; i < 14; i++) {
            if (i != 6 && i != 7) // sensor 6 & 7 tidak punya counter
                sensor_count[i] = 1 - sensor_count[i];
        }
        sensor_inverted = 1;
        noise_counter = 0;
    } else {
        if (g_line_param == 0 && noise_counter == 1) {
            // Simpan ke buffer kedua (kalibrasi/referensi)
            // Invert ke buffer 0x19fc-0x1a1c
            bitmask = 0x3FFF - bitmask;
            sensor_inverted = 1;
        }
        noise_counter = 0;
    }

    // Filter noise pattern
    if (is_noise_pattern(bitmask)) {
        if (g_line_param == 0) noise_counter++;
        return bitmask;
    }

    // Cek neighbor validity
    uint16_t neighbor = get_neighbor(bitmask);
    if (bitmask != neighbor) {
        return bitmask; // valid, langsung return
    }

    // Increment noise counter
    if (g_line_param     == 0) noise_counter++;
    return bitmask;
}

// ─────────────────────────────────────────
//  AUTO DETECT LINE TYPE
// ─────────────────────────────────────────

LineColor detect_line_type(uint16_t bitmask, uint8_t active_count) {
    uint16_t left_edge  = bitmask & 0x001F;
    uint16_t center     = (bitmask >> 5) & 0x0F;
    uint16_t right_edge = (bitmask >> 9) & 0x1F;

    if (center > 0 && left_edge == 0 && right_edge == 0 && active_count < 6) {
        return LINE_PUTIH;
    }
    if ((left_edge > 0 || right_edge > 0) && active_count > 3) {
        return LINE_HITAM;
    }
    return g_config.line;
}

// ─────────────────────────────────────────
//  INPUT ERROR — PID setpoint
//  Konversi bitmask sensor ke nilai error (-24 sampai +24)
// ─────────────────────────────────────────

int input_error_thin(uint16_t s) {
    switch (s) {
        case 0x0001: return -24; case 0x0002: return -20;
        case 0x0003: return -22; case 0x0004: return -16;
        case 0x0006: return -18; case 0x0007: return -20;
        case 0x0008: return -12; case 0x000c: return -14;
        case 0x000e: return -16; case 0x000f: return -18;
        case 0x0010: return  -8; case 0x0018: return -10;
        case 0x001c: return -12; case 0x001e: return -14;
        case 0x0020: return  -4; case 0x0030: return  -6;
        case 0x003c: return -10; case 0x0038: return  -8;
        case 0x0040: return  -1; case 0x0060: return  -2;
        case 0x0070: return  -4; case 0x0078: return  -6;
        case 0x00c0: return   0; case 0x01c0: return   0;
        case 0x00e0: return  -1; case 0x00f0: return  -2;
        case 0x0080: return  +1; case 0x0100: return  +4;
        case 0x0180: return  +2; case 0x0200: return  +8;
        case 0x0300: return  +6; case 0x0400: return +12;
        case 0x0600: return +10; case 0x0700: return  +8;
        case 0x0780: return  +6; case 0x0800: return +16;
        case 0x0c00: return +14; case 0x0e00: return +12;
        case 0x0f00: return +10; case 0x1000: return +20;
        case 0x1800: return +18; case 0x1c00: return +16;
        case 0x1e00: return +14; case 0x2000: return +24;
        case 0x3000: return +22; case 0x3800: return +20;
        case 0x3c00: return +18;
        default:     return (int)g_pv_out;
    }
}

int input_error_thick(uint16_t s) {
    switch (s) {
        case 0x007f: return +20; case 0x00ff: return +16;
        case 0x01ff: return +12; case 0x03ff: return  +8;
        case 0x07ff: return  +4; case 0x0fff: return  +2;
        case 0x1fff: return  +1; case 0x3fff: return   0;
        case 0x3fc0: return -16; case 0x3fe0: return -12;
        case 0x3ff0: return  -8; case 0x3ff8: return  -4;
        case 0x3ffc: return  -2; case 0x3ffe: return  -1;
        case 0x3f80: return -20;
        default:     return -(int)g_pv_out;
    }
}

int input_error_left(uint16_t s) {
    switch (s) {
        case 0x0000: return +20; case 0x0001: return +16;
        case 0x0003: return +12; case 0x0007: return  +8;
        case 0x000f: return  +4; case 0x001f: return  +2;
        case 0x003f: return  +1; case 0x007f: return   0;
        case 0x00ff: return  -1; case 0x01ff: return  -2;
        case 0x03ff: return  -4; case 0x07ff: return  -8;
        case 0x0fff: return -12; case 0x1fff: return -16;
        case 0x3fff: return -20;
        default:     return -(int)g_pv_out;
    }
}

int input_error_right(uint16_t s) {
    switch (s) {
        case 0x0000: return -20; case 0x2000: return -16;
        case 0x3000: return -12; case 0x3800: return  -8;
        case 0x3c00: return  -4; case 0x3e00: return  -2;
        case 0x3f00: return  -1; case 0x3f80: return   0;
        case 0x3fc0: return  +1; case 0x3fe0: return  +2;
        case 0x3ff0: return  +8; case 0x3ff8: return  +4;
        case 0x3ffc: return +12; case 0x3ffe: return +16;
        case 0x3fff: return +20;
        default:     return (int)g_pv_out;
    }
}

int input_error(uint16_t s) {
    int result;
    switch (g_mode_flag) {
        case 1: result =  input_error_thin(s);  break;
        case 2: result = -input_error_thick(s); break;
        case 3: result = -input_error_left(s);  break;
        case 4: result = -input_error_right(s); break;
        default: result = input_error_thin(s);  break;
    }
    g_pv_out = result;
    return result;
}