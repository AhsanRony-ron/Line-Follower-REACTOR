# 🤖 REACTOR — Line Follower Robot

Firmware robot **Line Follower** tingkat kompetisi berbasis mikrokontroler **STM32F103C8 (Blue Pill)**. Dibangun menggunakan **PlatformIO** + **Arduino Framework** dengan arsitektur modular yang mendukung dua mode operasi: **PID following standar** dan **mode counter/checkpoint** untuk menavigasi trek kompleks secara otonom.

---

## ⚠️ IMPORTANT

Jika ingin melihat dokumentasi cara setting counter mode, bisa di cek file [`usage-docs.md`](usage-docs.md).
di README ini tidak ada penjelasan mengenai cara setting counter mode.

---


## 📑 Daftar Isi

- [Fitur Utama](#-fitur-utama)
- [Spesifikasi Hardware](#-spesifikasi-hardware)
- [Wiring / Pin Mapping](#-wiring--pin-mapping)
- [Arsitektur Software](#-arsitektur-software)
- [Struktur Proyek](#-struktur-proyek)
- [Cara Memulai](#-cara-memulai)
- [Sistem Menu & Navigasi](#-sistem-menu--navigasi)
- [Mode Operasi](#-mode-operasi)
- [Sistem EEPROM & Memory Slot](#-sistem-eeprom--memory-slot)
- [Parameter Counter](#-parameter-counter)
- [Sistem Trigger](#-sistem-trigger)
- [Sensor & Kalibrasi](#-sensor--kalibrasi)
- [Kontribusi](#-kontribusi)

---

## ✨ Fitur Utama

- **14 sensor garis** via analog multiplexer — resolusi tinggi untuk deteksi posisi
- **Kontrol PID (Kp + Kd)** dengan error mapping berbasis lookup table
- **Mode Counter** — state machine sekuensial hingga 100 counter + 10 checkpoint
- **Sistem menu interaktif** pada layar OLED 128×64 dengan 6 tombol navigasi
- **15 slot memori EEPROM** — simpan/muat konfigurasi lengkap di lapangan
- **Rotary encoder** pada kedua roda untuk pengukuran jarak presisi
- **Mirror mode** — balik jalur trek secara otomatis tanpa perlu program ulang counter
- **Kalibrasi sensor otomatis** dengan visualisasi bar real-time pada OLED
- **Monitoring baterai** dengan low-pass filter dan indikator persentase
- **Hardware Check** — tes motor dan encoder langsung dari menu

---

## 🔧 Spesifikasi Hardware

| Komponen | Detail |
|---|---|
| **MCU** | STM32F103C8 (Blue Pill), 72 MHz, 64KB Flash, 20KB RAM |
| **Sensor Garis** | 14× sensor IR analog, multiplexed via 2 IC MUX 8-channel |
| **Driver Motor** | 2× H-Bridge (PWM via Timer1 & Timer4) |
| **Encoder** | 2× Rotary encoder (quadrature, interrupt-driven) |
| **Display** | OLED SSD1306 128×64 (I2C, alamat `0x3C`) |
| **Penyimpanan** | EEPROM AT24C256 (I2C, alamat `0x50`, 32KB) |
| **Input** | 6× push button (active LOW, internal pull-up) |
| **Baterai** | 3S LiPo (9.9V – 12.6V), dipantau via voltage divider (10kΩ/1kΩ) |
| **LED Indikator** | 2× LED (LCD backlight + timer indicator) |

---

## 🔌 Wiring / Pin Mapping

### Sensor (Multiplexer)

| Pin STM32 | Fungsi |
|---|---|
| `PA0` | ADC Channel 0 (sensor 0–7) |
| `PA1` | ADC Channel 1 (sensor 8–13) |
| `PA2` | MUX Select A |
| `PA3` | MUX Select B |
| `PA4` | MUX Select C |

### Motor

| Pin STM32 | Fungsi | Timer |
|---|---|---|
| `PA9` | Motor Kiri PWM1 | TIM1_CH2 |
| `PA10` | Motor Kiri PWM2 | TIM1_CH3 |
| `PB8` | Motor Kanan PWM1 | TIM4_CH3 |
| `PB9` | Motor Kanan PWM2 | TIM4_CH4 |

### Encoder

| Pin STM32 | Fungsi |
|---|---|
| `PB12` | Encoder Kiri A |
| `PB13` | Encoder Kiri B |
| `PB14` | Encoder Kanan A |
| `PB15` | Encoder Kanan B |

### Tombol (Active LOW, INPUT_PULLUP)

| Pin STM32 | Fungsi |
|---|---|
| `PB5` | Next (→) |
| `PB4` | Back (←) |
| `PA12` | Up (↑) |
| `PA11` | Down (↓) |
| `PA15` | Save (✓) |
| `PB3` | X (✗) |

### Lainnya

| Pin STM32 | Fungsi |
|---|---|
| `PA5` | Sensor tegangan baterai |
| `PA8` | OLED Reset |
| `PC14` | LED LCD |
| `PC13` | LED Timer |
| `PB6` | I2C SCL (OLED + EEPROM) |
| `PB7` | I2C SDA (OLED + EEPROM) |

---

## 🏗 Arsitektur Software

```
┌─────────────────────────────────────────────────┐
│                  main.cpp                       │
│         setup() → loop() → screen_standby()     │
│              ┌──────┴──────┐                    │
│         Mode Normal    Mode Counter             │
└──────┬──────────────────────┬───────────────────┘
       │                      │
  ┌────▼────┐          ┌──────▼───────┐
  │ pid.h   │          │mode_counter.h│
  │ PD Ctrl │          │ State Machine│
  └────┬────┘          └──────┬───────┘
       │                      │
  ┌────▼──────────────────────▼────┐
  │          sensor.h              │
  │  scan → threshold → bitmask   │
  │  → input_error() → PV         │
  └────────────────┬───────────────┘
                   │
       ┌───────────┼───────────┐
  ┌────▼───┐  ┌────▼───┐ ┌────▼─────┐
  │motor.h │  │display.h│ │eeprom.h  │
  │set_pwm │  │U8g2 GUI │ │AT24C256  │
  └────────┘  └─────────┘ └──────────┘
                   │
              ┌────▼────┐
              │ menu.h  │
              │UI Logic │
              └─────────┘
```

Alur data utama:

1. **Sensor** → 14 channel ADC dibaca via multiplexer → dibandingkan threshold → menghasilkan **bitmask 14-bit**
2. **Bitmask** → di-decode oleh `input_error()` menjadi nilai **error (-24 s/d +24)**
3. **PID** → `calc_pid()` menghitung output **P + D** dari error
4. **Motor** → PWM dikirim ke H-Bridge via `set_motors(LOUT, ROUT)`

---

## 📂 Struktur Proyek

```
Line-Follower-REACTOR/
├── platformio.ini          # Konfigurasi PlatformIO (board, lib, flags)
├── src/
│   └── main.cpp            # Entry point: setup(), loop(), timer ISR
├── include/
│   ├── config.h            # Pin definitions, hardware constants, MUX LUT
│   ├── types.h             # Struct & enum: GlobalConfig, CounterParam, dll.
│   ├── hardware.h          # GPIO init, PWM init, voltage reader, encoder ISR
│   ├── motor.h             # Motor control: set_motors(), helpers (maju, mundur, belok)
│   ├── sensor.h            # ADC read, scan, noise filter, input_error() LUT
│   ├── pid.h               # PID calculation & mode_normal()
│   ├── eeprom.h            # I2C EEPROM r/w, slot management, save/load
│   ├── display.h           # OLED rendering: standby, running, calibrate, menus
│   ├── menu.h              # UI logic: navigasi, edit, kalibrasi, hcheck
│   └── mode_counter.h      # State machine: counter loop, decision execution
└── arsip/                  # File backup / arsip lama
```

---

## 🚀 Cara Memulai

### Prasyarat

- [PlatformIO](https://platformio.org/) (IDE atau CLI)
- [ST-Link V2](https://www.st.com/en/development-tools/st-link-v2.html) programmer
- Driver ST-Link terinstall

### Build & Upload

```bash
# Clone repository
git clone https://github.com/<username>/Line-Follower-REACTOR.git
cd Line-Follower-REACTOR

# Build
pio run

# Upload via ST-Link
pio run --target upload

# Monitor serial (9600 baud)
pio device monitor
```

### Dependensi Library

| Library | Versi | Fungsi |
|---|---|---|
| [U8g2](https://github.com/olikraus/u8g2) | `^2.36.14` | Driver OLED SSD1306 |
| stm32duino core | built-in | HardwareTimer, I2C, GPIO |

---

## 📟 Sistem Menu & Navigasi

### Layar Standby

Layar utama saat robot dinyalakan. Menampilkan:
- **CP** — Checkpoint yang dipilih (CP:00 = mulai dari awal)
- **N** — Counter berikutnya dari checkpoint
- **Mode** — NORMAL atau COUNTER
- **Slot** — Memory slot aktif
- **Battery** — Tegangan dan persentase baterai
- **Sensor bar** — Visualisasi real-time 14 sensor

**Kontrol di layar standby:**

| Tombol | Fungsi |
|---|---|
| **Save** | START — Jalankan robot |
| **Up/Down** | Pilih Checkpoint (0–10) |
| **Next** | Buka Menu 1 (Config) |
| **Back** | Buka Menu 2 (Counter/CP) |
| **X** | Kalibrasi sensor cepat |

### Menu 1 — Konfigurasi Global

| Item | Keterangan | Range |
|---|---|---|
| Calibrate | Masuk mode kalibrasi sensor | — |
| Mode | NORMAL / COUNTER | 2 opsi |
| Speed | Kecepatan awal (launch speed) | 0–255 |
| PID | Kp dan Kd (tekan X untuk toggle) | 0–255 masing-masing |
| Line | Jenis garis: AUTO / HITAM / PUTIH | 3 opsi |
| Periode | Interval timer ISR (ms) | 1–9999 |
| T-Blank | Delay awal sebelum counter 0 | 0–9999 (×10ms) |
| PWM_F | Frekuensi PWM motor | 0.0–20.0 kHz |
| Mirror | Mode cermin (balik kiri-kanan) | YES / NO |
| H-Check | Tes motor dan encoder | — |
| M_Slot | Pilih memory slot aktif | 0–14 |

### Menu 2 — Counter & Data

| Item | Keterangan |
|---|---|
| Counter | Edit parameter counter 0–99 |
| Check Point | Edit checkpoint 1–10 |
| Copy Mem | Salin slot ke slot lain |
| Delete | Hapus slot aktif (reset ke default) |
| Reset | Factory reset semua slot |

---

## 🏎 Mode Operasi

### Mode Normal

Line following standar menggunakan kontrol **PD (Proportional-Derivative)**:

```
error    = -input_error(sensor_bitmask)
out_p    = error × Kp
out_d    = (error - last_error) × Kd
LOUT     = speed + out_p + out_d
ROUT     = speed - out_p - out_d
```

Robot mengikuti garis tanpa henti sampai tombol **Save** atau **X** ditekan.

### Mode Counter

Mode lanjutan berbasis **state machine sekuensial**. Setiap counter memiliki parameter tersendiri (speed, Kp, trigger, decision, dll). Alur eksekusi:

```
START
  │
  ▼
T-Blank (delay awal, following dengan speed_mode global)
  │
  ▼
┌─────── LOOP COUNTER (C0 → C99) ───────┐
│                                         │
│  1. Following dengan speed1→speed2      │
│     (ramp selama timer counter)         │
│                                         │
│  2. Setelah timer habis, cek trigger    │
│     counter berikutnya (sensor/timer)   │
│                                         │
│  3. Jika trigger match → eksekusi       │
│     decision counter berikutnya:        │
│     • LOST  — lanjut tanpa aksi         │
│     • FREE  — motor bebas (PWM manual)  │
│     • LEFT  — belok kiri + seek garis   │
│     • RIGHT — belok kanan + seek garis  │
│     • STOP  — rem, tampilkan waktu      │
│                                         │
│  4. Reset timer → lanjut counter        │
│     berikutnya                          │
└─────────────────────────────────────────┘
```

**Checkpoint** memungkinkan robot melompat ke counter tertentu saat START, berguna untuk:
- Mengulang bagian trek tertentu saat debugging
- Melanjutkan dari titik tertentu tanpa mulai dari awal

---

## 💾 Sistem EEPROM & Memory Slot

EEPROM AT24C256 (32KB) dibagi menjadi **15 slot** (0–14), masing-masing menyimpan:

| Offset | Ukuran | Data |
|---|---|---|
| `0x0000` | 14 bytes | `GlobalConfig` (mode, speed, PID, line, dll.) |
| `0x000E` | 1900 bytes | `CounterParam[100]` (19 bytes × 100 counter) |
| `0x076E` | 14 bytes | Sensor threshold (hasil kalibrasi) |
| `0x077C` | 30 bytes | `CheckpointParam[10]` (3 bytes × 10 CP) |
| **Total** | **~1959 bytes/slot** | |

**Master pointer** disimpan di alamat `0x7A60–0x7A61` untuk menandai slot aktif dan validation marker (`0xAA`).

Fitur manajemen slot:
- **Save** — simpan konfigurasi aktif ke EEPROM
- **Load** — muat konfigurasi dari slot yang dipilih (auto saat boot)
- **Copy** — salin satu slot ke slot lain
- **Delete** — hapus slot (isi 0xFF) lalu tulis default
- **Reset** — factory reset semua slot

---

## ⚙ Parameter Counter

Setiap counter (C0–C99) memiliki parameter berikut:

| Parameter | Tipe | Keterangan |
|---|---|---|
| `timer` | `uint16_t` | Durasi counter (×periode ms) |
| `speed1` | `uint8_t` | Kecepatan awal (ramp start) |
| `speed2` | `uint8_t` | Kecepatan akhir (ramp end) |
| `kp` | `uint8_t` | Kp khusus counter ini |
| `trigger` | `uint16_t` | Bitmask sensor trigger (14-bit) |
| `decision` | `Decision` | Aksi: LOST / FREE / LEFT / RIGHT / STOP |
| `delay_type` | `DelayType` | Tipe delay: A (non-blocking) atau B (blocking) |
| `delay_ms` | `uint16_t` | Durasi delay aksi (ms) |
| `belok_l` | `int16_t` | PWM roda kiri saat belok (-255 s/d 255) |
| `belok_r` | `int16_t` | PWM roda kanan saat belok (-255 s/d 255) |
| `Encd_l` | `int16_t` | Target tick encoder kiri |
| `Encd_r` | `int16_t` | Target tick encoder kanan |
| `Encd_d` | `int16_t` | Target tick encoder untuk belok (enc otomatis r/l)|
| `Line_C` | `LineCounterMode` | Mode pembacaan garis per counter |

> **Catatan:** Counter 0 (C0) hanya memiliki **Timer**, **Speed**, dan **Kp**. Parameter decision, trigger, dan belok hanya tersedia mulai C1.

---

## 🎯 Sistem Trigger

Trigger menentukan **kapan** robot berpindah dari counter saat ini ke counter berikutnya. Jenis trigger:

| Trigger | Kode | Keterangan |
|---|---|---|
| `TMR` | `0x3FFF` | Pindah otomatis setelah timer habis |
| `L1`–`L5` | bit 9–13 | Sensor kiri tertentu aktif |
| `R1`–`R5` | bit 0–4 | Sensor kanan tertentu aktif |
| `LR1`–`LR5` | bit kiri+kanan | Kedua sisi aktif bersamaan |
| `BLK` | `0x0000` | Semua sensor blank (tidak ada garis) |
| `ENC` | `0x3FFE` | Trigger berbasis encoder tick |

Pada mode **Mirror**, trigger secara otomatis di-flip (kiri ↔ kanan) sehingga konfigurasi satu arah bisa dipakai untuk arah sebaliknya.

---

## 📊 Sensor & Kalibrasi

### Prinsip Kerja

1. 14 sensor IR analog dibaca melalui **2 IC multiplexer 8-channel** (total 16 channel, 14 digunakan)
2. Setiap pembacaan menghasilkan nilai 8-bit (0–255)
3. Nilai dibandingkan dengan **threshold** (hasil kalibrasi) untuk menghasilkan status aktif/tidak aktif
4. Status 14 sensor di-encode menjadi **bitmask 14-bit** (`g_sensor_out`)
5. Bitmask di-decode oleh `input_error()` menggunakan **lookup table** menjadi nilai error -24 s/d +24

### Mode Input Error

| Mode | Fungsi | Keterangan |
|---|---|---|
| `thin` | `input_error_thin()` | Garis tipis (default) — posisi berdasarkan cluster aktif |
| `thick` | `input_error_thick()` | Garis tebal — deteksi berdasarkan tepi |
| `left` | `input_error_left()` | Garis tepi kiri |
| `right` | `input_error_right()` | Garis tepi kanan |

Mode ini di-set per counter melalui parameter `Line_C` (Line Counter Mode).

### Kalibrasi

1. Dari layar standby, tekan **X** untuk masuk mode kalibrasi
2. Geser robot bolak-balik melewati garis hitam dan area putih
3. Sistem merekam nilai **high** (hitam) dan **low** (putih) setiap sensor
4. Threshold dihitung otomatis: `thresh = (high - low_offset) / 2 + low_offset`
5. Tekan **Save** untuk menyimpan ke EEPROM

Offset noise filter default: **50%** dari range (dapat diubah via `SENSOR_OFFSET_PCT`).

---

## 🤝 Kontribusi

### Cara Berkontribusi

1. **Fork** repository ini
2. Buat **branch** fitur baru: `git checkout -b fitur/nama-fitur`
3. **Commit** perubahan: `git commit -m "Tambah fitur XYZ"`
4. **Push** ke branch: `git push origin fitur/nama-fitur`
5. Buat **Pull Request**

### Panduan Kode

- Semua kode ditulis dalam **C++ (Arduino Framework)** dengan target **STM32**
- Header files di folder `include/` menggunakan pola **header-only** (implementasi langsung di `.h`)
- Gunakan prefix `g_` untuk variabel global
- Komentar dalam bahasa **Indonesia** (konsisten dengan kode yang ada)
- Struct yang disimpan ke EEPROM **wajib** menggunakan `#pragma pack(1)` dan `__attribute__((packed))`
- Pastikan perubahan struct di-sync dengan layout EEPROM agar backward compatible

### Build & Test

```bash
# Build saja (tanpa upload)
pio run

# Upload ke board
pio run --target upload

# Clean build
pio run --target clean
```

---

## 📜 Lisensi

Dibuat oleh **Ron** — *Created by Ron with salving 4 AI*
And **Hakaisha** — *With only 1 AI, and just making this file*

---

> **💡 Tips:** Saat debugging di lapangan, manfaatkan **Memory Slot** untuk menyimpan beberapa konfigurasi berbeda (misal: trek A di slot 0, trek B di slot 1). Gunakan fitur **Copy Mem** untuk menduplikasi konfigurasi yang sudah jalan sebagai backup sebelum melakukan tuning.
