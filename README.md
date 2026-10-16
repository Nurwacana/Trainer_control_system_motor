# Trainer Control System Motor

Sistem kontrol kecepatan motor DC loop tertutup (*closed-loop velocity control*) berbasis mikrokontroler Arduino Uno dan antarmuka pemantauan desktop berbasis Python (PyQt5 & PyQtGraph). Proyek ini dirancang untuk keperluan pengujian, penelitian, dan modul pembelajaran sistem kontrol PID serta feedforward secara *real-time*.

---

## Fitur Utama

- **Kontrol PID & Feedforward Presisi**:
  - Algoritma PID dengan pembatasan integral (*anti-windup*) dan *zero-crossing reset*.
  - Kompensasi *feedforward linear* ($u_{ff} = m \cdot \text{target} + c$).
  - Filter turunan (*derivative filtering* dengan parameter $\tau$).
  - *Sample time* PID dinamis yang dapat disesuaikan langsung dari GUI secara *real-time*.
- **Protokol Komunikasi Serial Handal**:
  - Protokol biner berkecepatan tinggi (115200 baud).
  - Frame dilengkapi *Start-of-Frame* (SOF), *End-of-Frame* (EOF), dan validasi integritas checksum **CRC32 (IEEE 802.3)**.
  - Mekanisme *fail-safe*: Motor otomatis berhenti jika komunikasi serial terputus melebihi batas waktu aman (*watchdog timeout*).
- **Monitoring & Telemetri Real-Time**:
  - Pengukuran arus dan tegangan bus motor secara presisi via sensor **INA219** (I2C).
  - Tampilan lokal status RPM, setpoint, dan konsumsi daya pada LCD karakter 16x2 (I2C).
- **Antarmuka Desktop GUI Modern**:
  - Plotting grafik *real-time* respons RPM dan sinyal kendali PWM dengan latensi rendah menggunakan **PyQtGraph**.
  - Panel tuning parameter langsung ($K_p$, $K_i$, $K_d$, $m$, $c$, *sample time*, target RPM).
  - Multi-threaded serial worker untuk memastikan GUI responsif tanpa jeda (*non-blocking*).

---

## Arsitektur & Spesifikasi Perangkat Keras

| Komponen | Spesifikasi / Model | Pin Arduino | Keterangan |
| :--- | :--- | :--- | :--- |
| **Mikrokontroler** | Arduino Uno (ATmega328P) | - | Clock 16 MHz, Baudrate 115200 |
| **Driver Motor** | H-Bridge Driver | D5 (PWM), D7 (DIR1), D8 (DIR2) | Sinyal PWM kontrol kecepatan & arah putar |
| **Rotary Encoder** | Optik / Kuadratur 600 PPR | D2 (INT0 / Channel A), D3 (INT1 / Channel B) | Decoding 4x (*quadrature*) = 2400 ticks/putaran |
| **Sensor Arus/Tegangan** | Adafruit INA219 | SDA (A4), SCL (A5) | Alamat I2C: `0x40` |
| **Layar Status** | LCD Karakter 16x2 I2C | SDA (A4), SCL (A5) | Alamat I2C: `0x27` |

---

## Struktur Repositori

```text
Trainer_control_system_motor/
├── .gitignore                          # Konfigurasi file yang diabaikan Git
├── README.md                           # Dokumentasi proyek
├── ardu_control_motor_fw/              # Proyek Firmware PlatformIO
│   ├── platformio.ini                  # Konfigurasi board, dependensi library, & baudrate
│   ├── include/                        # File header pendukung
│   ├── lib/                            # Pustaka kustom / pihak ketiga
│   └── src/
│       ├── main.cpp                    # Logika utama firmware (PID, sensor, interrupt, serial)
│       └── SerialPacketComm.h          # Driver pustaka komunikasi paket serial & CRC32
└── GUI/                                # Aplikasi Desktop GUI
    ├── GUI_new.py                      # Aplikasi GUI utama (PyQt5 + PyQtGraph)
    └── GUI_Serial.py                   # Implementasi alternatif antarmuka serial GUI
```

---

## Protokol Komunikasi Serial

Komunikasi data antara Arduino dan PC menggunakan format *packed struct* biner:

- **Baudrate**: `115200 bps`
- **SOF Marker**: `0xAA, 0xBB, 0xCC`
- **EOF Marker**: `0xCC, 0xBB, 0xAA`
- **Checksum**: CRC32 dihitung terhadap seluruh byte payload sebelum field CRC.

### 1. Host ke Arduino (Command Frame)
Mengirimkan setpoint, gain kendali, dan parameter waktu:
- SOF (`3 byte`)
- `target_rpm` (`float`, 4 byte)
- `ff_m`, `ff_c` (`float`, 8 byte)
- `kp`, `ki`, `kd` (`float`, 12 byte)
- `sample_time` (`float`, 4 byte)
- CRC32 (`uint32`, 4 byte)
- EOF (`3 byte`)

### 2. Arduino ke Host (Telemetry Frame)
Mengirimkan status pembacaan sensor secara periodik:
- SOF (`3 byte`)
- `current_rpm` (`float`, 4 byte)
- `current_pwm` (`float`, 4 byte)
- `current_voltage` (`float`, 4 byte)
- `current_current` (`float`, 4 byte)
- CRC32 (`uint32`, 4 byte)
- EOF (`3 byte`)

---

## Panduan Penggunaan

### 1. Flash Firmware Arduino

Pastikan telah menginstal [VS Code](https://code.visualstudio.com/) bersama ekstensi [PlatformIO IDE](https://platformio.org/).

1. Buka folder `ardu_control_motor_fw` di VS Code PlatformIO.
2. Hubungkan papan Arduino Uno ke port USB komputer.
3. Jalankan build dan upload melalui PlatformIO CLI atau tombol taskbar:
   ```bash
   pio run -t upload
   ```
4. Pantau serial monitor jika diperlukan:
   ```bash
   pio device monitor
   ```

### 2. Menjalankan Aplikasi GUI

Pastikan lingkungan Python (versi 3.8+) telah terinstal di komputer.

1. Masuk ke direktori GUI dan instal dependensi pustaka:
   ```bash
   pip install PyQt5 pyqtgraph pyserial numpy
   ```

2. Jalankan aplikasi GUI:
   ```bash
   python GUI/GUI_new.py
   ```

3. Pada jendela antarmuka:
   - Pilih Port COM yang terhubung ke Arduino.
   - Klik tombol **Hubungkan** / **Connect**.
   - Atur setpoint RPM dan parameter PID/Feedforward pada panel kendali.
   - Respon motor dan grafik telemetri akan langsung terbarui secara *real-time*.

---

## Lisensi & Kontribusi

Proyek ini dikembangkan untuk tujuan riset dan edukasi trainer kendali motor. Silakan buat *issue* atau ajukan *pull request* untuk pengembangan lebih lanjut.
