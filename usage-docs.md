# Dokumentasi Line Follower Robot

Referensi parameter counter, menu, dan navigasi.

---

## Tipe Trigger

Setiap counter memiliki parameter **trigger** yang menentukan kapan transisi ke counter berikutnya terjadi.

| Trigger | Perilaku |
|---|---|
| `TMR` (Timer) | Transisi langsung setelah durasi timer habis |
| `ENC` (Encoder tick) | Transisi langsung setelah jumlah tick encoder tercapai |
| Sensor | Sensor yang ditentukan harus menyentuh garis terlebih dahulu sebelum transisi terjadi |

> **Catatan:** `tick` dan `timer` bersifat mutually exclusive. Jika `tick > 0`, parameter tick otomatis menimpa timer. Jika `tick = 0`, gunakan timer.

---

## Tipe Keputusan (DEC)

### DEC_LEFT / DEC_RIGHT

Mode belok kiri atau kanan, dengan dua tipe perilaku setelah belok.

| Parameter | Keterangan |
|---|---|
| `timer` | Waktu tunggu sebelum trigger counter berikutnya aktif (satuan: timer × periode global) |
| `tick` | Pengganti timer berbasis encoder. Jika `tick = 0`, gunakan timer |
| `spd` | Aktif setelah fase `dly` selesai, saat robot kembali following garis dan timer mulai dihitung. `speed1` = kecepatan awal; `speed2` = kecepatan akhir. Naik linear |
| `dly` | **Type A:** belok selama durasi delay, lalu jalan terus sampai menemukan garis. **Type B:** belok selama durasi delay, lalu lanjut jalan biasa. Durasi tersedia dalam ms dan encoder tick. Kecepatan belok mengikuti parameter motor R/L |
| `kp` | Nilai P (proportional) PID untuk counter ini |
| `motor` | Output PWM ke motor selama fase `dly` |
| `line` | Jenis garis yang diikuti (putih / hitam / auto) |
| `trigger` | Lihat tabel tipe trigger di atas |

---

### DEC_FREE

Mode gerak bebas. Parameter `motor` meng-override PID selama FREE mode berlangsung.

| Parameter | Keterangan |
|---|---|
| `timer` | Durasi FREE mode berlangsung |
| `tick` | Pengganti timer berbasis encoder. Jika `tick = 0`, gunakan timer |
| `spd` | `speed1` tidak dipakai. Langsung pakai `speed2` setelah FREE mode berakhir |
| `kp` | Nilai P PID untuk counter ini |
| `motor` | Output PWM ke motor selama FREE mode — meng-override PID |
| `line` | Jenis garis yang diikuti |
| `trigger` | Lihat tabel tipe trigger di atas |

---

### DEC_STOP

Mode berhenti. LF tetap line-following selama countdown, lalu berhenti.

| Parameter | Keterangan |
|---|---|
| `timer` | Durasi countdown sebelum stop (jika trigger TMR). Selama durasi ini LF tetap following line |
| `tick` | Pengganti timer jika trigger ENC |
| `spd` | `speed1` = kecepatan awal; `speed2` = kecepatan akhir. Naik linear selama countdown. `speed1` diabaikan jika trigger sensor (langsung berhenti) |
| `kp` | Nilai P PID untuk counter ini |
| `line` | Jenis garis yang diikuti |
| `trigger` | Jika trigger sensor: robot langsung berhenti saat sensor menyentuh garis, tanpa countdown |

---

### DEC_LOST

Mode line following PID biasa, tidak ada perilaku khusus.

| Parameter | Keterangan |
|---|---|
| `timer` | Durasi LOST mode berlangsung |
| `tick` | Pengganti timer berbasis encoder. Jika `tick = 0`, gunakan timer |
| `spd` | `speed1` = kecepatan awal; `speed2` = kecepatan akhir. Naik linear |
| `kp` | Nilai P PID untuk counter ini |
| `line` | Jenis garis yang diikuti |
| `trigger` | Lihat tabel tipe trigger di atas |

---

## Menu 1 — Pengaturan Umum

| Item | Fungsi |
|---|---|
| Calibrate | Kalibrasi sensor. Taruh di lantai dulu → lewatkan semua sensor ke garis → save |
| Mode | **Normal:** line following PID biasa. **Counter:** jalankan sequence counter |
| Speed | Kecepatan di mode normal. Juga kecepatan startup mode counter (sebelum counter ke-0) selama durasi T-Blank |
| PID | Parameter KP dan KD untuk sistem PD |
| Line | Jenis garis di mode normal: putih / hitam / auto |
| Periode | Pengali waktu untuk timer counter. Default: 10 ms (rekomendasi) |
| T-Blank | Durasi kecepatan startup sebelum counter mulai berjalan |
| PWM_F | Frekuensi PWM ke motor. Default: 10 kHz |
| Mirror | Membalik semua parameter counter secara otomatis (untuk arena yang simetris/mirror) |
| H-Check | Cek motor dan encoder. Kecepatan motor = nilai Speed |
| M-Slot | Slot memori untuk arena berbeda. Setiap slot menyimpan semua parameter secara independen |

---

## Menu 2 — Manajemen Counter & Memori

| Item | Fungsi |
|---|---|
| Counter | Masuk ke pengaturan counter |
| Checkpoint | Mulai dari counter tertentu sesuai posisi arena. Durasi delay per-checkpoint diset secara independen |
| Copy Mem | Salin isi satu memory slot ke slot tujuan |
| Delete | Hapus semua parameter pada slot memori yang sedang aktif |
| Reset | Reset total semua memori |

---

## Navigasi Tombol

### Layar Standby

| Tombol | Fungsi |
|---|---|
| Save | Tahan dan lepas untuk mulai jalan |
| Up / Down | Naik/turun checkpoint yang digunakan (`CP:00` = nomor checkpoint, `N:C00` = counter yang dilanjutkan) |
| X | Masuk langsung ke kalibrasi sensor |
| Next | Masuk Menu 1 |
| Back | Masuk Menu 2 |

### Menu 1

| Tombol | Fungsi |
|---|---|
| Save | Simpan dan kembali ke standby |
| Up / Down | Navigasi item atas/bawah |
| X | Masuk kalibrasi / H-Check, atau pilih field yang akan diedit (ditandai `[ ]`) |
| Next / Back | Naik/turun nilai parameter yang sedang diedit |

### Menu 2

| Tombol | Fungsi |
|---|---|
| Save | Kembali ke standby |
| Up / Down | Navigasi item atas/bawah |
| X / Back / Next | Masuk ke menu pilihan |

### Menu Counter

| Tombol | Fungsi |
|---|---|
| Save | Simpan parameter dan kembali ke Menu 2 |
| Up / Down | Navigasi item atas/bawah |
| X | Pilih field yang akan diedit (ditandai `[ ]`) |
| Next / Back | Naik/turun nilai parameter yang sedang diedit |
