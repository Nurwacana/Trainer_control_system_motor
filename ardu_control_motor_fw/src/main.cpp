#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_INA219.h>

#define ENCODER_PPR 600
#define PIN_ENC_A 2   // INT0
#define PIN_ENC_B 3   // INT1

#define PIN_PWM   5
#define PIN_DIR1  7
#define PIN_DIR2  8

// Resolusi 4x quadrature: PPR * 4 = 2400 ticks/rev
const float TOTAL_PULSES_PER_REV = ENCODER_PPR * 4.0f;

// Objek Komunikasi & Periferal
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_INA219 ina219(0x40);

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float setpoint;
    float prev_meas;
    float integral;

    float out_min;
    float out_max;

    float sample_time;
    uint32_t last_time;

    _Bool first_time;

    float error;

    float out;

    float tau;
    float prev_derivative;

    int max_integral;

    _Bool using_zero_crossing_integral_reset;
} PID_t;

typedef enum {
    PID_KP,
    PID_KI,
    PID_KD,
    PID_SAMPLE_TIME,
    PID_OUT,
    PID_MAX_INTEGRAL,
    PID_TAU,
} PID_Parameter_t;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float out_min, float out_max, float sample_time_sec)
{
	if(!pid->first_time) {
	    pid->Kp = Kp;
	    pid->Ki = Ki;
	    pid->Kd = Kd;

	    pid->prev_meas = 0.0f;
	    pid->integral   = 0.0f;

	    pid->out_min = out_min;
	    pid->out_max = out_max;

	    pid->sample_time = sample_time_sec;
	    pid->last_time = millis();

	    pid->out = 0;
	    pid->first_time = 1;
	}
}

void PID_Set_Parameter(PID_t *pid, PID_Parameter_t param, float value)
{
    switch(param) {
        case PID_KP:
            pid->Kp = value;
            break;
        case PID_KI:
            pid->Ki = value;
            break;
        case PID_KD:
            pid->Kd = value;
            break;
        case PID_SAMPLE_TIME:
            pid->sample_time = value;
            break;
        case PID_OUT:
            pid->out = value;
            break;
        case PID_MAX_INTEGRAL:
            pid->max_integral = (int)value;
            break;
        case PID_TAU:
            pid->tau = value;
            break;
    }
}

void PID_Set_Setpoint(PID_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

void PID_Compute(PID_t *pid, const float measurement, const float feedforward)
{
    uint32_t now = millis();
    float dt = (now - pid->last_time) / 1000.0f;

    pid->error = pid->setpoint - measurement;

    // Jika belum mencapai sample time → jangan hitung PID
    if (dt < pid->sample_time)
        return;     // keluar, output tetap pid->out

    // Update timestamp
    pid->last_time = now;

    // Integral
    pid->integral += pid->error * dt;

    if (pid->Ki > 0.0f) {
        float max_i = pid->out_max / pid->Ki;
        float min_i = pid->out_min / pid->Ki;
        if (pid->integral > max_i) pid->integral = max_i;
        else if (pid->integral < min_i) pid->integral = min_i;
    }

    // Anti-windup
    if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    else if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

    // === TAMBAHAN LOGIKA ZERO CROSSING ===
    if(pid->using_zero_crossing_integral_reset)
    {
        // Hitung error pada siklus sebelumnya (asumsi setpoint tidak berubah drastis)
        float prev_error = pid->setpoint - pid->prev_meas;

        // Cek perubahan tanda secara komparasi langsung
        if ((pid->error > 0.0f && prev_error < 0.0f) || (pid->error < 0.0f && prev_error > 0.0f))
        {
            pid->integral = 0.0f; // Reset akumulasi integral
        }
    }
    // =====================================

    // Derivative (Menggunakan -dP/dt)
    float derivative = -(measurement - pid->prev_meas) / dt;

    // PID Output
    float output =  pid->Kp * pid->error
                  + pid->Ki * pid->integral
                  + pid->Kd * derivative
                  + feedforward;

    // Limit Output
    if (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;

    // Simpan status untuk iterasi berikutnya
    pid->prev_meas = measurement;
    pid->out = output;
}

volatile long encoderTicks = 0;

static const int8_t ENC_STATES[] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
};
static volatile uint8_t encStateHistory = 0;

struct EncoderSample {
  long deltaTicks;
  unsigned long deltaMicros;
};

void handleEncoderISR() {
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  
  encStateHistory = ((encStateHistory << 2) | (a << 1) | b) & 0x0F;
  int8_t direction = ENC_STATES[encStateHistory];
  
  if (direction != 0) {
    encoderTicks += direction;
  }
}

EncoderSample get_enc() {
  static long lastTicks = 0;
  static unsigned long lastMicros = 0;

  noInterrupts();
  long currentTicks = encoderTicks;
  unsigned long currentMicros = micros();
  interrupts();

  EncoderSample sample;
  sample.deltaTicks = currentTicks - lastTicks;
  sample.deltaMicros = currentMicros - lastMicros;

  lastTicks = currentTicks;
  lastMicros = currentMicros;

  return sample;
}

void setMotor(float controlSignal) {
  int pwmValue = (int)abs(controlSignal);
  pwmValue = constrain(pwmValue, 0, 255);

  digitalWrite(PIN_DIR1, controlSignal < 0.0f);
  digitalWrite(PIN_DIR2, controlSignal > 0.0f);

  analogWrite(PIN_PWM, pwmValue);
}

PID_t pid;
uint32_t prev_pid;
uint32_t prev_telemetry;
uint32_t prev_INA;
uint32_t prev_lcd; // Pewaktu refresh LCD

struct {
  float rpm;
  float output;
  float voltage;
  float current;
} motor;

#define MARKER_LEN 3

const uint8_t EXPECTED_SOF[MARKER_LEN] = {0xAA, 0xBB, 0xCC};
const uint8_t EXPECTED_EOF[MARKER_LEN] = {0xCC, 0xBB, 0xAA};

typedef struct {
  uint8_t sof[MARKER_LEN];

  float target_rpm;
  float ff_m, ff_c;
  float kp, ki, kd, sample_time;

  uint32_t crc;
  uint8_t eof[MARKER_LEN];
} __attribute__((packed)) SerialPacket_Rx_t;

typedef struct {
  uint8_t sof[MARKER_LEN];

  float current_rpm;
  float current_pwm;
  float current_voltage;
  float current_current;

  uint32_t crc;
  uint8_t eof[MARKER_LEN];
} __attribute__((packed)) SerialPacket_Tx_t;

SerialPacket_Tx_t Tx_data;
SerialPacket_Rx_t Rx_data;

uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xEDB88320UL;
      } else {
        crc >>= 1;
      }
    }
  }
  return ~crc;
}

#define RX_FRAME_SIZE sizeof(SerialPacket_Rx_t)

uint8_t Rx_buffer[RX_FRAME_SIZE];
uint8_t Rx_sorted[RX_FRAME_SIZE];
uint8_t rx_count = 0;
uint8_t rx_sync_index = 0;

uint32_t last_packet_true;

static void reorder_rx_frame() {
  for (uint8_t i = 0; i < RX_FRAME_SIZE; i++) {
    uint8_t src_idx = (uint8_t)((rx_sync_index + i) % RX_FRAME_SIZE);
    Rx_sorted[i] = Rx_buffer[src_idx];
  }
}

static _Bool validate_rx_frame() {
  for (uint8_t i = 0; i < RX_FRAME_SIZE; i++) {
    _Bool eof_matched = true;

    for (uint8_t b = 0; b < MARKER_LEN; b++) {
      uint8_t idx = (uint8_t)((i + b) % RX_FRAME_SIZE);
      if (Rx_buffer[idx] != EXPECTED_EOF[b]) {
        eof_matched = false;
        break;
      }
    }

    if (eof_matched) {
      rx_sync_index = (uint8_t)((i + MARKER_LEN) % RX_FRAME_SIZE);

      reorder_rx_frame();

      SerialPacket_Rx_t *packet = (SerialPacket_Rx_t *)Rx_sorted;

      if (memcmp(packet->sof, EXPECTED_SOF, MARKER_LEN) == 0) {
        uint32_t calculated_crc = calculateCRC32(
          (const uint8_t *)packet,
          offsetof(SerialPacket_Rx_t, crc)
        );

        if (calculated_crc == packet->crc) {
          return true;
        }
      }
    }
  }

  return false;
}

void readSerialPacket() {
  while (Serial.available() > 0) {
    Rx_buffer[rx_count++] = (uint8_t)Serial.read();

    if (rx_count >= RX_FRAME_SIZE) {
      if (validate_rx_frame()) {
        memcpy(&Rx_data, Rx_sorted, sizeof(SerialPacket_Rx_t));

        PID_Set_Parameter(&pid, PID_KP, Rx_data.kp);
        PID_Set_Parameter(&pid, PID_KI, Rx_data.ki);
        PID_Set_Parameter(&pid, PID_KD, Rx_data.kd);
        PID_Set_Parameter(&pid, PID_SAMPLE_TIME, Rx_data.sample_time);
        PID_Set_Setpoint(&pid, Rx_data.target_rpm);

        last_packet_true = millis();
        
        rx_count = 0;
      } else {
        memmove(&Rx_buffer[0], &Rx_buffer[1], RX_FRAME_SIZE - 1);
        rx_count = RX_FRAME_SIZE - 1;
      }
    }
  }
}

// Fungsi pembantu memperbarui tampilan LCD secara rapi tanpa pergeseran layout
void updateLCD() {
  char line1[17];
  char line2[17];

  // Baris 1: R:xxxx P:xxx (RPM dan Nilai PWM)
  // Contoh: "R: 450.2 P: 180" atau "R:-120.5 P:-120"
  char str_rpm[8];
  dtostrf(motor.rpm, 6, 1, str_rpm); // 6 karakter total, 1 digit desimal
  snprintf(line1, sizeof(line1), "R:%s P:%4d", str_rpm, (int)motor.output);

  // Baris 2: V:xx.xV I:xxxxmA (Tegangan Bus dan Arus mA)
  // Contoh: "V:12.1V I: 350mA"
  char str_volt[6];
  dtostrf(motor.voltage, 4, 1, str_volt);
  snprintf(line2, sizeof(line2), "V:%sV I:%5dmA", str_volt, (int)motor.current);

  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR1, OUTPUT);
  pinMode(PIN_DIR2, OUTPUT);

  uint8_t initialA = digitalRead(PIN_ENC_A);
  uint8_t initialB = digitalRead(PIN_ENC_B);
  encStateHistory = (initialA << 1) | initialB;

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), handleEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), handleEncoderISR, CHANGE);

  Wire.begin();
  Wire.setClock(400000UL); // 400 kHz Fast-Mode I2C

  // Inisialisasi LCD 16x2
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready...");

  if (!ina219.begin()) {
    // Handle error inisialisasi INA219 jika terlepas
  } else {
    ina219.setCalibration_32V_2A();
    // Bacaan awal langsung disalin ke variabel terfilter
    motor.voltage = ina219.getBusVoltage_V();
    motor.current = ina219.getCurrent_mA();
  }

  get_enc();

  PID_Init(&pid, 3.0f, 0.1f, 0.00f, -255.0f, 255.0f, 0.01f);
  PID_Set_Parameter(&pid, PID_MAX_INTEGRAL, 10000.0f);
  
  prev_pid = millis();
  prev_INA = millis();
  prev_telemetry = millis();
  prev_lcd = millis();
}

void loop() {
  uint32_t now = millis();

  // 1. Loop Kontrol PID (100 Hz / 10 ms)
  if(now - prev_pid > 10) {
    prev_pid = now;
    
    PID_Set_Setpoint(&pid, Rx_data.target_rpm);
    EncoderSample current_sample = get_enc();

    motor.rpm = 0.0f;
    if (current_sample.deltaMicros > 0) {
      motor.rpm = -((float)current_sample.deltaTicks * 60000000.0f) / 
            ((float)current_sample.deltaMicros * TOTAL_PULSES_PER_REV);
    }

    float feedfowrward_pwm = Rx_data.target_rpm * Rx_data.ff_m + Rx_data.ff_c;

    PID_Compute(&pid, motor.rpm, feedfowrward_pwm);
    motor.output = pid.out;
  }

  // 2. Pembacaan Sensor INA219 (50 Hz / 20 ms)
  if (now - prev_INA >= 20) {
    prev_INA = now;

    float raw_voltage = ina219.getBusVoltage_V();
    float raw_current = ina219.getCurrent_mA();

    // Koefisien filter smoothing (EMA)
    const float alpha_v = 0.12f; // Filter halus untuk tegangan bus
    const float alpha_i = 0.20f; // Filter sedang untuk dinamika arus motor

    // Eksekusi Low-Pass Filter
    motor.voltage = (alpha_v * raw_voltage) + ((1.0f - alpha_v) * motor.voltage);
    motor.current = (alpha_i * raw_current) + ((1.0f - alpha_i) * motor.current);

    // Deadband pembersih noise sisa saat motor benar-benar idle
    if (fabs(motor.current) < 2.0f && abs((int)motor.output) == 0) {
      motor.current = 0.0f;
    }
  }

  // 3. Pembacaan Paket Serial Rx
  readSerialPacket();

  // 4. Watchdog Proteksi Aktuator (500 ms)
  if(now - last_packet_true > 500) {
    setMotor(0); 
    PID_Set_Setpoint(&pid, 0);
  } else {
    setMotor(motor.output);
  }

  // 5. Telemetri Serial Tx (50 Hz / 20 ms)
  if(now - prev_telemetry >= 20) {
    prev_telemetry = now;

    Tx_data.sof[0] = 0xAA;
    Tx_data.sof[1] = 0xBB;
    Tx_data.sof[2] = 0xCC;

    Tx_data.current_rpm = motor.rpm;
    Tx_data.current_pwm = motor.output;
    Tx_data.current_current = motor.current;
    Tx_data.current_voltage = motor.voltage;

    Tx_data.crc = calculateCRC32((const uint8_t *)&Tx_data, offsetof(SerialPacket_Tx_t, crc));

    Tx_data.eof[0] = 0xCC;
    Tx_data.eof[1] = 0xBB;
    Tx_data.eof[2] = 0xAA;

    Serial.write((const uint8_t *)&Tx_data, sizeof(Tx_data));
  }

  // 6. Pembaruan LCD 16x2 (10 Hz / 100 ms)
  if(now - prev_lcd >= 100) {
    prev_lcd = now;
    updateLCD();
  }
}