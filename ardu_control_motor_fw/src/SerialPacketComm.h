#pragma once

#include <Arduino.h>
#include <string.h>

// --- Definisi Struct Payload ---
typedef struct {
    uint32_t tick;
    float current_rpm;
    float target_rpm_pwm;
    float current_mA;
    float bus_voltage_V;
} __attribute__((packed)) SerialPacketToSend;

typedef struct {
    uint32_t tick;
    uint8_t mode;
    float target_rpm_pwm;
    float feedforward_m;
    float feedforward_c;
    float kp;
    float ki;
    float kd;
    float controll_period_ms;
} __attribute__((packed)) SerialPacketToReceive;

// --- Frame Container ---
template <typename TPayload>
struct __attribute__((packed)) SerialFrame {
    uint8_t sof[3];    // Start-of-Frame sync (contoh: 0xAA, 0xBB, 0xCC)
    uint8_t len;       // Panjang payload (sizeof(TPayload))
    TPayload data;     // Data struct
    uint32_t crc;      // CRC32 checksum dari (len + data)
};

// --- Class Transceiver Serial ---
template <typename TxPayload, typename RxPayload>
class SerialPacketComm {
public:
    static constexpr size_t RX_FRAME_SIZE = sizeof(SerialFrame<RxPayload>);

private:
    Stream* port;
    const uint8_t sof_bytes[3];

    // Buffer mentah dan buffer yang sudah di-reorder
    uint8_t raw_buffer[RX_FRAME_SIZE];
    uint8_t sorted_buffer[RX_FRAME_SIZE];
    uint16_t rx_raw_count;

    // Output Data
    RxPayload latest_rx_data;
    bool new_data_flag;

    // Fast Bitwise CRC32 (IEEE 802.3 Standard)
    uint32_t calculateCRC32(const uint8_t* buffer, size_t size) {
        uint32_t crc = 0xFFFFFFFF;
        for (size_t i = 0; i < size; i++) {
            crc ^= buffer[i];
            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 1) {
                    crc = (crc >> 1) ^ 0xEDB88320;
                } else {
                    crc >>= 1;
                }
            }
        }
        return ~crc;
    }

    // Meluruskan buffer circular berdasarkan titik sync index yang ditemukan
    void reorderFrame(uint16_t sync_index) {
        for (size_t i = 0; i < RX_FRAME_SIZE; i++) {
            size_t source_index = (sync_index + i) % RX_FRAME_SIZE;
            sorted_buffer[i] = raw_buffer[source_index];
        }
    }

    // Memvalidasi SOF, length, dan CRC32
    bool validateAndExtractFrame() {
        for (size_t i = 0; i < RX_FRAME_SIZE; i++) {
            size_t idx0 = i;
            size_t idx1 = (i + 1) % RX_FRAME_SIZE;
            size_t idx2 = (i + 2) % RX_FRAME_SIZE;

            // Cari 3 byte SOF berurutan secara circular
            if (raw_buffer[idx0] == sof_bytes[0] &&
                raw_buffer[idx1] == sof_bytes[1] &&
                raw_buffer[idx2] == sof_bytes[2]) {

                reorderFrame(i);

                SerialFrame<RxPayload>* frame = reinterpret_cast<SerialFrame<RxPayload>*>(sorted_buffer);

                // Validasi panjang payload
                if (frame->len != sizeof(RxPayload)) {
                    continue; // Header kebetulan cocok tapi field panjang salah
                }

                // Validasi CRC32 terhadap (len + data)
                uint32_t computed_crc = calculateCRC32(&(frame->len), sizeof(frame->len) + sizeof(RxPayload));
                
                if (computed_crc == frame->crc) {
                    latest_rx_data = frame->data;
                    new_data_flag = true;
                    return true;
                }
            }
        }
        return false;
    }

public:
    SerialPacketComm(Stream& serial_port, uint8_t s0 = 0xAA, uint8_t s1 = 0xBB, uint8_t s2 = 0xCC)
        : port(&serial_port), sof_bytes{s0, s1, s2}, rx_raw_count(0), new_data_flag(false) {
        memset(raw_buffer, 0, sizeof(raw_buffer));
        memset(sorted_buffer, 0, sizeof(sorted_buffer));
    }

    void begin() {
        resetParser();
    }

    void resetParser() {
        rx_raw_count = 0;
    }

    // --- Pengiriman Frame Serial ---
    void send(const TxPayload& payload) {
        SerialFrame<TxPayload> frame;
        frame.sof[0] = sof_bytes[0];
        frame.sof[1] = sof_bytes[1];
        frame.sof[2] = sof_bytes[2];
        frame.len = sizeof(TxPayload);
        frame.data = payload;

        frame.crc = calculateCRC32(&(frame.len), sizeof(frame.len) + sizeof(TxPayload));

        port->write(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame));
    }

    // --- Parser Non-Blocking (Pola Buffer & Reorder) ---
    bool processRx() {
        bool frame_received = false;

        while (port->available() > 0) {
            raw_buffer[rx_raw_count++] = port->read();

            // Setelah buffer terkumpul penuh 1 frame, evaluasi frame
            if (rx_raw_count >= RX_FRAME_SIZE) {
                if (validateAndExtractFrame()) {
                    rx_raw_count = 0; // Reset buffer setelah sukses mendapatkan 1 frame utuh
                    frame_received = true;
                } else {
                    // Geser data 1 byte ke kiri jika buffer penuh tapi invalid (sliding window fallback)
                    memmove(&raw_buffer[0], &raw_buffer[1], RX_FRAME_SIZE - 1);
                    rx_raw_count = RX_FRAME_SIZE - 1;
                }
            }
        }

        return frame_received;
    }

    bool available() const {
        return new_data_flag;
    }

    RxPayload read() {
        new_data_flag = false;
        return latest_rx_data;
    }
};