// === TX: ESP32-C3 SuperMini + MPU6050 -> ESP-NOW pan/tilt ===
// Reads IMU, computes yaw (gyro Z) + pitch (complementary), broadcasts angles.
// Board: ESP32C3 Dev Module

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Wire.h>

// ====== USER CONFIG ======
#define WIFI_CHANNEL        1           // Must match RX
#define I2C_SDA_PIN         5           // using 5 (SDA), 6 (SCL) per your change
#define I2C_SCL_PIN         6
#define I2C_FREQ_HZ         400000

#define RECENTER_BTN_PIN    0           // momentary to GND (active LOW)

// Angle processing
#define ALPHA               0.98f       // complementary filter factor (gyro trust)
#define PITCH_LIMIT_DEG     45.0f       // mechanical limit for camera tilt
#define YAW_LIMIT_DEG       90.0f       // soft limit for pan (-90..+90)

// Output mapping (center offsets if your camera is physically offset)
#define PITCH_CENTER_DEG    0.0f
#define YAW_CENTER_DEG      0.0f

// Optional: invert axes (set to -1 to invert, +1 to normal)
#define PITCH_SIGN          +1
#define YAW_SIGN            +1

// Failsafe/telemetry
#define TX_RATE_HZ          100         // send rate (~10 ms)
#define SERIAL_BAUD         115200

// ====== MPU6050 register defs (no extra libs) ======
#define MPU6050_ADDR        0x68
#define REG_PWR_MGMT_1      0x6B
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B

// Scale factors (±2g, ±250 dps)
static const float ACC_SCALE = 16384.0f;   // LSB/g
static const float GYR_SCALE = 131.0f;     // LSB/(°/s)

// ====== Packet ======
typedef struct __attribute__((packed)) {
  uint32_t seq;
  int16_t yaw_deg;    // -180..+180
  int16_t pitch_deg;  // -90..+90
} HeadPkt;

volatile uint32_t seq_no = 0;

// ====== State ======
float pitch_deg = 0.0f;      // filtered pitch
float yaw_deg   = 0.0f;      // integrated yaw (minus offset)
float yaw_offset = 0.0f;     // recenter reference
unsigned long last_us;

// Gyro zero offsets (computed at startup)
float gx_bias = 0, gy_bias = 0, gz_bias = 0;

// Recenter button state
bool btnPrev = true;               // pull-up -> idle HIGH
unsigned long btnDebounceMs = 0;
const unsigned long BTN_DEBOUNCE = 200; // ms

// ====== I2C helpers ======
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuReadBytes(uint8_t startReg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start
  size_t r = Wire.requestFrom(MPU6050_ADDR, (uint8_t)n);
  for (size_t i=0; i<n && Wire.available(); ++i) buf[i] = Wire.read();
  return r == n;
}

bool mpuInit() {
  if (!mpuWrite(REG_PWR_MGMT_1, 0x00)) return false;  // wake
  delay(50);
  mpuWrite(REG_SMPLRT_DIV, 0x04);   // ~200 Hz
  mpuWrite(REG_CONFIG, 0x03);       // DLPF ~44Hz
  mpuWrite(REG_GYRO_CONFIG, 0x00);  // ±250 dps
  mpuWrite(REG_ACCEL_CONFIG, 0x00); // ±2 g
  delay(50);
  return true;
}

bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t buf[14];
  if (!mpuReadBytes(REG_ACCEL_XOUT_H, buf, 14)) return false;
  ax = (int16_t)((buf[0]<<8)|buf[1]);
  ay = (int16_t)((buf[2]<<8)|buf[3]);
  az = (int16_t)((buf[4]<<8)|buf[5]);
  gx = (int16_t)((buf[8]<<8)|buf[9]);
  gy = (int16_t)((buf[10]<<8)|buf[11]);
  gz = (int16_t)((buf[12]<<8)|buf[13]);
  return true;
}

// ====== Gyro bias (hold still!) ======
void calibrateGyro(size_t samples=500) {
  long gx_sum=0, gy_sum=0, gz_sum=0;
  for (size_t i=0; i<samples; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    if (mpuReadRaw(ax, ay, az, gx, gy, gz)) {
      gx_sum += gx; gy_sum += gy; gz_sum += gz;
    }
    delay(2);
  }
  gx_bias = (float)gx_sum / samples;
  gy_bias = (float)gy_sum / samples;
  gz_bias = (float)gz_sum / samples;
}

// ====== ESP-NOW ======
void initESPNOW() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) delay(10);
  }
  // Broadcast peer
  esp_now_peer_info_t peer{};
  memset(&peer, 0, sizeof(peer));
  for (int i=0;i<6;++i) peer.peer_addr[i] = 0xFF;
  peer.ifidx = WIFI_IF_STA;
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

// ====== Utils ======
static inline int16_t clamp_i16(int v, int lo, int hi){
  if (v<lo) return lo;
  if (v>hi) return hi;
  return v;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  pinMode(RECENTER_BTN_PIN, INPUT_PULLUP);   // don't hold LOW at boot

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  if (!mpuInit()) {
    Serial.println("MPU init failed. Check wiring.");
    while (1) delay(10);
  }
  Serial.println("Calibrating gyro... hold still ~1s");
  calibrateGyro(500);
  Serial.printf("Gyro biases: gx=%.1f gy=%.1f gz=%.1f\n", gx_bias, gy_bias, gz_bias);

  initESPNOW();
  last_us = micros();
}

void loop() {
  // timing
  unsigned long now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;
  if (dt <= 0) dt = 1e-3f;
  last_us = now_us;

  // read sensors
  int16_t ax, ay, az, gx, gy, gz;
  if (!mpuReadRaw(ax, ay, az, gx, gy, gz)) return;

  // convert
  float ax_g = ax / ACC_SCALE;
  float ay_g = ay / ACC_SCALE;
  float az_g = az / ACC_SCALE;
  float gx_dps = (gx - gx_bias) / GYR_SCALE;
  float gy_dps = (gy - gy_bias) / GYR_SCALE;
  float gz_dps = (gz - gz_bias) / GYR_SCALE;

  // accel-based pitch (aircraft-ish: positive pitch when looking up)
  float acc_pitch = atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)) * 180.0f / PI;

  // integrate gyro for pitch & yaw (MPU orientation dependent)
  float pitch_pred = pitch_deg + (gy_dps) * dt; // gyro Y -> pitch
  float yaw_pred   = yaw_deg   + (gz_dps) * dt; // gyro Z -> yaw (will offset below)

  // complementary filter on pitch, yaw pure gyro with offset (recenter)
  pitch_deg = ALPHA * pitch_pred + (1.0f - ALPHA) * acc_pitch;

  // Apply recenter offset to yaw
  yaw_deg   = yaw_pred - yaw_offset;

  // ----- Recenter button (active LOW) with debounce & edge detect -----
  bool btnNow = (digitalRead(RECENTER_BTN_PIN) == LOW);
  unsigned long ms = millis();
  if (btnNow != btnPrev && (ms - btnDebounceMs) > BTN_DEBOUNCE) {
    btnDebounceMs = ms;
    if (btnNow) {
      // capture current yaw reference BEFORE clamp/center/invert
      yaw_offset = yaw_pred;     // make current heading zero
      Serial.println("Yaw recentered");
    }
    btnPrev = btnNow;
  }

  // Apply centers, signs, and limits
  float out_pitch = (PITCH_SIGN) * (pitch_deg + PITCH_CENTER_DEG);
  float out_yaw   = (YAW_SIGN)   * (yaw_deg   + YAW_CENTER_DEG);

  if (out_pitch >  PITCH_LIMIT_DEG) out_pitch =  PITCH_LIMIT_DEG;
  if (out_pitch < -PITCH_LIMIT_DEG) out_pitch = -PITCH_LIMIT_DEG;
  if (out_yaw   >  YAW_LIMIT_DEG)   out_yaw   =  YAW_LIMIT_DEG;
  if (out_yaw   < -YAW_LIMIT_DEG)   out_yaw   = -YAW_LIMIT_DEG;

  // pack & send
  HeadPkt pkt;
  pkt.seq = seq_no++;
  pkt.yaw_deg   = (int16_t)roundf(out_yaw);
  pkt.pitch_deg = (int16_t)roundf(out_pitch);

  uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  esp_now_send(bcast, (uint8_t*)&pkt, sizeof(pkt));

  // debug @ ~20 Hz
  static uint32_t dbg_t = 0;
  if (millis() - dbg_t > 50) {
    dbg_t = millis();
    Serial.printf("seq:%lu yaw:%d pitch:%d  | rawYawOff:%.1f gz:%.1f gy:%.1f accP:%.1f\n",
      (unsigned long)pkt.seq, (int)pkt.yaw_deg, (int)pkt.pitch_deg,
      yaw_offset, gz_dps, gy_dps, acc_pitch);
  }

  // pacing to ~TX_RATE_HZ
  static const uint32_t period_ms = (1000 / TX_RATE_HZ);
  static uint32_t last_ms = 0;
  uint32_t now_ms = millis();
  if (now_ms - last_ms < period_ms) {
    delay(period_ms - (now_ms - last_ms));
  }
  last_ms = millis();
}
