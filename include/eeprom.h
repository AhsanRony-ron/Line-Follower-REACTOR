#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "types.h"

// ─────────────────────────────────────────
//  EXTERN
// ─────────────────────────────────────────
extern GlobalConfig     g_config;
extern CounterParam     g_counter[COUNTER_MAX];
extern CheckpointParam  g_checkpoint[CP_MAX];
extern uint8_t          g_sensor_thresh[SENSOR_COUNT];

// ─────────────────────────────────────────
//  LAYOUT EEPROM
//
//  Per slot (1958 bytes):
//    0x0000 - 0x000D : GlobalConfig       (14 bytes)
//    0x000E - 0x076D : CounterParam x100  (1900 bytes, 19 bytes each)
//    0x076E - 0x077B : Sensor thresh      (14 bytes)
//    0x077C - 0x079F : CheckpointParam x10(30 bytes, 3 bytes each)
//
//  7 slot total, mulai dari address 0:
//    Slot 0: 0x0000, Slot 1: 0x07A0, dst
// ─────────────────────────────────────────

#define EEPROM_OFFSET_CONFIG    0
#define EEPROM_OFFSET_COUNTER   (EEPROM_OFFSET_CONFIG  + sizeof(GlobalConfig))           // 15
#define EEPROM_OFFSET_SENSOR    (EEPROM_OFFSET_COUNTER + sizeof(CounterParam) * COUNTER_MAX)  // 15 + 1900 = 1915
#define EEPROM_OFFSET_CP        (EEPROM_OFFSET_SENSOR  + SENSOR_COUNT)                   // 1915 + 14 = 1929
#define EEPROM_SLOT_SIZE        (EEPROM_OFFSET_CP      + sizeof(CheckpointParam) * CP_MAX)    // 1929 + 30 = 1959

// ─────────────────────────────────────────
//  LOW LEVEL: BACA / TULIS BYTE
// ─────────────────────────────────────────

uint8_t eeprom_read_byte(uint16_t addr) {
    Wire.beginTransmission(EEPROM_DEVICE_ADDR);
    Wire.write((addr >> 8) & 0xFF);
    Wire.write((addr >> 0) & 0xFF);
    uint8_t status = Wire.endTransmission();
    
    if (status != 0) return 0xFF;
    
    delayMicroseconds(0); // ← dari delay(5) = 5000µs, jadi 100µs
    
    Wire.requestFrom((int)EEPROM_DEVICE_ADDR, (int)1);
    if (Wire.available()) return Wire.read();
    return 0xFF;
}

void eeprom_write_byte(uint16_t addr, uint8_t val) {
    Wire.beginTransmission(EEPROM_DEVICE_ADDR);
    Wire.write((addr >> 8) & 0xFF);
    Wire.write((addr >> 0) & 0xFF);
    Wire.write(val);
    uint8_t status = Wire.endTransmission();
    
    // Tunggu write cycle selesai (AT24C256 ~5-10ms)
    // Tambahan delay untuk memastikan write completion di STM32
    delay(15);
}

// baca block bytes ke buffer - OPTIMIZED untuk sequential read
// AT24C256 support sequential read setelah address setup
void eeprom_read_block(uint16_t addr, uint8_t* buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = eeprom_read_byte(addr + i);
    }
}

void eeprom_wait_ready() {
    // ACK polling: kirim address terus sampai EEPROM respond
    uint8_t timeout = 100;
    while (timeout--) {
        Wire.beginTransmission(EEPROM_DEVICE_ADDR);
        uint8_t err = Wire.endTransmission();
        if (err == 0) return;  // EEPROM siap
        delay(1);
    }
}

// tulis block bytes dari buffer dengan page write
// AT24C256: page size = 64 bytes, write cycle ~5ms
void eeprom_write_block(uint16_t addr, const uint8_t* buf, uint16_t len) {
    const uint16_t PAGE_SIZE = 64;
    uint16_t offset = 0;
    
    while (offset < len) {
        // hitung berapa bytes yang bisa ditulis di page ini
        uint16_t page_addr = addr + offset;
        uint16_t page_offset = page_addr % PAGE_SIZE;
        uint16_t bytes_in_page = PAGE_SIZE - page_offset;
        uint16_t chunk = min((uint16_t)(len - offset), bytes_in_page);
        
        // tulis chunk dalam satu page
        Wire.beginTransmission(EEPROM_DEVICE_ADDR);
        Wire.write((page_addr >> 8) & 0xFF);
        Wire.write((page_addr >> 0) & 0xFF);
        for (uint16_t i = 0; i < chunk; i++) {
            Wire.write(buf[offset + i]);
        }
        uint8_t status = Wire.endTransmission();
        
        eeprom_wait_ready();
        
        offset += chunk;
    }
}

// ─────────────────────────────────────────
//  HELPER: BASE ADDRESS SLOT
// ─────────────────────────────────────────

inline uint16_t slot_base(uint8_t slot) {
    return (uint16_t)slot * EEPROM_SLOT_SIZE;
}

// ─────────────────────────────────────────
//  SAVE
// ─────────────────────────────────────────

void eeprom_save_config(uint8_t slot) {
    eeprom_write_block(
        slot_base(slot) + EEPROM_OFFSET_CONFIG,
        (const uint8_t*)&g_config,
        sizeof(GlobalConfig)
    );
}

void eeprom_save_counter(uint8_t slot) {
    eeprom_write_block(
        slot_base(slot) + EEPROM_OFFSET_COUNTER,
        (const uint8_t*)g_counter,
        sizeof(CounterParam) * COUNTER_MAX
    );
}

void eeprom_save_sensor(uint8_t slot) {
    eeprom_write_block(
        slot_base(slot) + EEPROM_OFFSET_SENSOR,
        g_sensor_thresh,
        SENSOR_COUNT
    );
}

void eeprom_save_cp(uint8_t slot) {
    eeprom_write_block(
        slot_base(slot) + EEPROM_OFFSET_CP,
        (const uint8_t*)g_checkpoint,
        sizeof(CheckpointParam) * CP_MAX
    );
}

// save semua ke slot tertentu
void eeprom_save_all(uint8_t slot) {
    eeprom_save_config(slot);
    eeprom_save_counter(slot);
    eeprom_save_sensor(slot);
    eeprom_save_cp(slot);
}

// ─────────────────────────────────────────
//  LOAD
// ─────────────────────────────────────────

void eeprom_load_config(uint8_t slot) {
    eeprom_read_block(
        slot_base(slot) + EEPROM_OFFSET_CONFIG,
        (uint8_t*)&g_config,
        sizeof(GlobalConfig)
    );
}

void eeprom_load_counter(uint8_t slot) {
    eeprom_read_block(
        slot_base(slot) + EEPROM_OFFSET_COUNTER,
        (uint8_t*)g_counter,
        sizeof(CounterParam) * COUNTER_MAX
    );
}

void eeprom_load_sensor(uint8_t slot) {
    eeprom_read_block(
        slot_base(slot) + EEPROM_OFFSET_SENSOR,
        g_sensor_thresh,
        SENSOR_COUNT
    );
}

void eeprom_load_cp(uint8_t slot) {
    eeprom_read_block(
        slot_base(slot) + EEPROM_OFFSET_CP,
        (uint8_t*)g_checkpoint,
        sizeof(CheckpointParam) * CP_MAX
    );
}

// load semua dari slot tertentu
void eeprom_load_all(uint8_t slot) {
    eeprom_load_config(slot);
    eeprom_load_counter(slot);
    eeprom_load_sensor(slot);
    eeprom_load_cp(slot);
}

// ─────────────────────────────────────────
//  COPY SLOT
//  Copy slot aktif ke slot tujuan
// ─────────────────────────────────────────

void eeprom_copy_slot(uint8_t src, uint8_t dst) {
    uint8_t buf[32]; // baca & tulis 32 byte per iterasi
    uint16_t total = EEPROM_SLOT_SIZE;
    uint16_t offset = 0;

    while (offset < total) {
        uint16_t chunk = min((uint16_t)32, (uint16_t)(total - offset));
        eeprom_read_block(slot_base(src) + offset, buf, chunk);
        eeprom_write_block(slot_base(dst) + offset, buf, chunk);
        offset += chunk;
    }
}

// ─────────────────────────────────────────
//  LOAD DEFAULT
//  Isi g_config & g_counter dengan nilai default
//  (dipakai saat EEPROM kosong / setelah reset)
// ─────────────────────────────────────────

// isi RAM dengan nilai default
void load_defaults() {
    g_config.mode         = MODE_NORMAL;
    g_config.speed_mode   = DEFAULT_SPEED1;
    g_config.kp           = DEFAULT_KP;
    g_config.kd           = DEFAULT_KD;
    g_config.line         = DEFAULT_LINE;
    g_config.periode      = DEFAULT_PERIODE;
    g_config.t_blank      = 0;
    g_config.pwm_freq_khz = DEFAULT_PWM_FREQ;
    g_config.mem_slot     = 0;
    g_config.mirrored     = false;  // ← tambah ini

    for (uint8_t i = 0; i < COUNTER_MAX; i++) {
        g_counter[i].timer      = DEFAULT_TIMER;
        g_counter[i].speed1     = DEFAULT_SPEED1;
        g_counter[i].speed2     = DEFAULT_SPEED2;
        g_counter[i].kp         = DEFAULT_KP;
        g_counter[i].trigger    = DEFAULT_TRIGGER;  // ← fix, sebelumnya TRIGGER_TIMER
        g_counter[i].decision   = DEFAULT_DECISION;
        g_counter[i].delay_type = DEFAULT_DELAY_TYPE;
        g_counter[i].delay_ms   = DEFAULT_DELAY_MS;
        g_counter[i].motor_l    = DEFAULT_MOTOR_L;
        g_counter[i].motor_r    = DEFAULT_MOTOR_R;
        g_counter[i].Encd_l     = 0;
        g_counter[i].Encd_r     = 0;
        g_counter[i].Encd_b     = 0;
        g_counter[i].Line_C     = DEFAULT_LINE_COUNTER; 
    }

    for (uint8_t i = 0; i < CP_MAX; i++) {
        g_checkpoint[i].counter_pos = 0xFF;
        g_checkpoint[i].timer_cp   = 0;
    }

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        g_sensor_thresh[i] = 128;
    }
}

// ─────────────────────────────────────────
//  DELETE SLOT
//  Isi slot dengan 0xFF (EEPROM blank state)
// ─────────────────────────────────────────

void eeprom_delete_slot(uint8_t slot) {
    GlobalConfig    backup_config = g_config;
    CounterParam    backup_counter[COUNTER_MAX];
    memcpy(backup_counter, g_counter, sizeof(g_counter));
    CheckpointParam backup_cp[CP_MAX];
    memcpy(backup_cp, g_checkpoint, sizeof(g_checkpoint));

    load_defaults();
    g_config.mem_slot = slot;
    eeprom_save_all(slot);

    g_config = backup_config;
    memcpy(g_counter,    backup_counter, sizeof(g_counter));
    memcpy(g_checkpoint, backup_cp,      sizeof(g_checkpoint));
}
// ─────────────────────────────────────────
//  RESET TOTAL
//  Hapus semua slot (isi semua 0xFF)
//  Optimized: page write untuk kecepatan
// ─────────────────────────────────────────

void eeprom_reset_all() {
    for (uint8_t s = 0; s < EEPROM_SLOT_COUNT; s++) {
        eeprom_delete_slot(s);
    }
}


// tulis default ke SEMUA slot EEPROM
// dipanggil sekali saat factory reset atau EEPROM baru
void eeprom_write_defaults_all_slots() {
    load_defaults();
    for (uint8_t s = 0; s < EEPROM_SLOT_COUNT; s++) {
        g_config.mem_slot = s;   // tiap slot simpan mem_slot-nya sendiri
        eeprom_save_all(s);
    }
    g_config.mem_slot = 0;       // kembali ke slot 0
}

// ─────────────────────────────────────────
//  INIT
//  Load dari slot aktif, jika kosong/invalid load default
//  Verifikasi: cek beberapa byte pertama untuk memastikan data valid
//  PENTING: Validasi mem_slot tetap dalam range 0-6
//  OPTIMASI: Hanya load config dulu, biarkan user ambil data jika perlu
// ─────────────────────────────────────────

// ─────────────────────────────────────────
//  MASTER SLOT POINTER
//  Disimpan di dua byte terakhir AT24C256 (32768 bytes)
//  0x7FFE = index slot aktif
//  0x7FFF = validation marker (0xAA = valid)
// ─────────────────────────────────────────
#define EEPROM_MASTER_ADDR      0x7A60
#define EEPROM_VALID_ADDR       0x7A61
#define EEPROM_VALID_MARKER  0xAA

void eeprom_save_active_slot(uint8_t slot) {
    eeprom_write_byte(EEPROM_MASTER_ADDR, slot);
    eeprom_write_byte(EEPROM_VALID_ADDR,  EEPROM_VALID_MARKER);
}

void eeprom_init() {
    uint8_t marker = eeprom_read_byte(EEPROM_VALID_ADDR);

    if (marker != EEPROM_VALID_MARKER) {
        // EEPROM baru / belum pernah ditulis
        // Tulis default ke SEMUA slot sekaligus
        eeprom_write_defaults_all_slots();
        eeprom_save_active_slot(0);
    } else {
        // EEPROM valid — baca slot aktif
        uint8_t active = eeprom_read_byte(EEPROM_MASTER_ADDR);
        if (active >= EEPROM_SLOT_COUNT) active = 0;

        // Load SEMUA dari satu slot yang sama
        eeprom_load_all(active);

        // Pastikan mem_slot di RAM sesuai slot yang dimuat
        g_config.mem_slot = active;
    }
}

void eeprom_backup() {
    if (!Serial3.available()) return;

    char cmd = (char)Serial3.read();

    if (cmd == 'P') {
        Serial3.println("OK");

    } else if (cmd == 'R') {
        const uint16_t EEPROM_SIZE = 32768;

        // Kirim size (2 byte big-endian)
        Serial3.write((uint8_t)((EEPROM_SIZE >> 8) & 0xFF));
        Serial3.write((uint8_t)((EEPROM_SIZE >> 0) & 0xFF));

        // Baca byte-by-byte pakai eeprom_read_byte (sama seperti eeprom_read_block)
        for (uint16_t addr = 0; addr < EEPROM_SIZE; addr++) {
            Serial3.write(eeprom_read_byte(addr));
        }

        // End marker
        Serial3.write((uint8_t)0xDE);
        Serial3.write((uint8_t)0xAD);
        Serial3.flush();
    }
}