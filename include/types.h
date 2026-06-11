#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────
//  ENUMS
// ─────────────────────────────────────────

typedef uint8_t RobotMode;
#define MODE_NORMAL   ((RobotMode)0)
#define MODE_COUNTER  ((RobotMode)1)

typedef uint8_t LineColor;
#define LINE_AUTO     ((LineColor)0)
#define LINE_HITAM    ((LineColor)4)
#define LINE_PUTIH    ((LineColor)8)

typedef uint8_t Decision;
#define DEC_LOST        ((Decision)0)
#define DEC_FREE        ((Decision)1)
#define DEC_BELOK_KIRI  ((Decision)2)
#define DEC_BELOK_KANAN ((Decision)3)
#define DEC_STOP        ((Decision)4)

typedef uint8_t DelayType;
#define DELAY_A ((DelayType)0)
#define DELAY_B ((DelayType)1)

typedef uint8_t LineCounterMode;
#define AUTO_NORMAL ((LineCounterMode)0)
#define AUTO_CENTER ((LineCounterMode)1)
#define AUTO_LEFT   ((LineCounterMode)2)
#define AUTO_RIGHT  ((LineCounterMode)3)
#define BLACK_NORMAL ((LineCounterMode)4)
#define BLACK_CENTER ((LineCounterMode)5)
#define BLACK_LEFT   ((LineCounterMode)6)
#define BLACK_RIGHT  ((LineCounterMode)7)
#define WHITE_NORMAL ((LineCounterMode)8)
#define WHITE_CENTER ((LineCounterMode)9)
#define WHITE_LEFT   ((LineCounterMode)10)
#define WHITE_RIGHT  ((LineCounterMode)11)



// Trigger sensor — encoded sebagai bitmask 14-bit
// detect_timer = 0b11111111111111 (semua bit 1) = trigger hanya timer
#define TRIGGER_TIMER   0b11111111111111  // semua bit 1 = bypass
#define TRIGGER_TICK    0b11111111111110  // bit 1 = 0, reserved untuk tick
#define TRIGGER_BLANK   0

// ─────────────────────────────────────────
//  STRUCT PACKING
//  Gunakan #pragma pack(1) untuk memastikan tidak ada padding
//  saat save/load ke EEPROM
// ─────────────────────────────────────────
#pragma pack(push, 1)

// Parameter per counter
struct __attribute__((packed)) CounterParam {
    uint16_t  timer;        // target cacahan (×10ms)
    uint8_t   speed1;       // kecepatan saat timer berjalan
    uint8_t   speed2;       // kecepatan setelah timer habis
    uint8_t   kp;           // Kp khusus counter ini
    uint16_t  trigger;      // bitmask sensor trigger (14-bit)
    Decision  decision;     // aksi yang dieksekusi
    DelayType delay_type;   // A atau B
    uint16_t  delay_ms;     // durasi delay (pure ms)
    int16_t   belok_l;      // PWM roda kiri saat belok/lost
    int16_t   belok_r;      // PWM roda kanan saat belok/lost
    int16_t   Encd_l;       // Encoder Kiri
    int16_t   Encd_r;       // Encoder kanan
    int16_t   Encd_b;       // Encoder belok
    LineCounterMode Line_C; // Mode penghitung garis
};

// Parameter checkpoint
struct __attribute__((packed)) CheckpointParam {
    uint8_t  counter_pos;   // dipasang di counter ke-N, resume dari N+1
    uint16_t timer_cp;      // delay awal setelah resume (×10ms)
};

// Konfigurasi global (Menu 1)
struct __attribute__((packed)) GlobalConfig {
    RobotMode   mode;
    uint8_t     speed_mode;     // launch speed sebelum counter mulai
    uint8_t     kp;             // Kp mode normal
    uint8_t     kd;             // Kd global
    LineColor   line;
    uint16_t    periode;        // pembagi ISR: g_timer naik setiap periode×1ms
    uint16_t    t_blank;        // delay sebelum counter 0 (×10ms)
    float       pwm_freq_khz;   // frekuensi PWM motor (kHz)
    uint8_t     mem_slot;       // slot EEPROM aktif (0-6)
    bool        mirrored;       // mode mirror (C1+)
};

// ─────────────────────────────────────────
//  END STRUCT PACKING — Restore default alignment
// ─────────────────────────────────────────
#pragma pack(pop)

// ─────────────────────────────────────────
//  DEFAULT VALUES
// ─────────────────────────────────────────

#define DEFAULT_SPEED1      80
#define DEFAULT_SPEED2      80
#define DEFAULT_MAX_PWM     255
#define DEFAULT_KP          5
#define DEFAULT_KD          200
#define DEFAULT_TIMER       10      
#define DEFAULT_DELAY_MS    50
#define DEFAULT_BELOK_L     60
#define DEFAULT_BELOK_R     60
#define DEFAULT_FREE_L      50
#define DEFAULT_FREE_R      50
#define DEFAULT_LINE        LINE_AUTO
#define DEFAULT_TRIGGER     TRIGGER_TIMER
#define DEFAULT_DECISION    DEC_STOP
#define DEFAULT_DELAY_TYPE  DELAY_A
#define DEFAULT_PWM_FREQ    0.5f
#define DEFAULT_PERIODE     10
#define DEFAULT_LINE_COUNTER   AUTO_NORMAL


// Array limits
#define COUNTER_MAX         100
#define CP_MAX              10