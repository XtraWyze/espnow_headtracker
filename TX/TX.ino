// ESP32-C3 SuperMini — HEAD UNIT (TX)
// MPU6050 + QMC5883L -> Madgwick -> send yaw/pitch via ESP-NOW

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>   // for wifi_tx_info_t on newer cores
#include <esp_now.h>
#include <MadgwickAHRS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <QMC5883LCompass.h>

#define I2C_SDA   5
#define I2C_SCL   6
#define PIN_RECENTER 0   // button to GND (optional)

Adafruit_MPU6050 mpu;
QMC5883LCompass qmc;
Madgwick filter;

struct Packet {
  float yaw_deg;
  float pitch_deg;
} tx;

// === PUT YOUR RECEIVER'S MAC HERE ===
uint8_t PEER_MAC[6] = { 0x98, 0x3D, 0xAE, 0x51, 0xB3, 0x54 };

float yaw_offset = 0.0f;
float yaw_s = 0, pitch_s = 0;
const float ALPHA = 0.18f;   // smoothing (0..1)
unsigned long lastMicros = 0;

// New ESP-NOW send callback signature (Arduino-ESP32 v3 / IDF v5+)
void onSend(const wifi_tx_info_t* /*info*/, esp_now_send_status_t /*status*/) {
  // Optional debug:
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SEND OK" : "SEND FAIL");
}

void recenterYaw(float currentYaw) {
  yaw_offset += currentYaw;  // make current heading zero
}

void setup() {
  pinMode(PIN_RECENTER, INPUT_PULLUP);
  Serial.begin(115200);
  delay(150);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); // start at 100 kHz for reliability

  // MPU6050
  if (!mpu.begin()) { Serial.println("MPU6050 not found!"); while(1) delay(10); }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // QMC5883L
  qmc.init();
  // Optional tuning:
  // qmc.setRange(2);         // 2 or 8 Gauss
  // qmc.setSamplingRate(50); // 10/50/100/200 Hz
  // qmc.setOversampling(64); // 64/128/256/512
  // qmc.setMode(0x01);       // continuous

  // Wi-Fi STA + ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while(1) delay(10);
  }
  esp_now_register_send_cb(onSend);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = 0;      // current channel
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Add peer failed");
    while(1) delay(10);
  }

  // Madgwick
  filter.begin(100.0f);  // nominal sample rate
  lastMicros = micros();

  Serial.println("TX ready (ESP-NOW, QMC5883L@0x0D, MPU6050@0x68)");
}

void loop() {
  // Read MPU
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  // Read QMC (API returns ints, no arguments)
  qmc.read();
  int mx_i = qmc.getX();
  int my_i = qmc.getY();
  int mz_i = qmc.getZ();

  // Convert to floats (units arbitrary; Madgwick normalizes internally)
  float mx = (float)mx_i;
  float my = (float)my_i;
  float mz = (float)mz_i;

  // dt (tracked; this Madgwick impl doesn't require explicit dt)
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if (dt <= 0) dt = 0.01f;

  // Gyro deg/s -> rad/s
  float gx = radians(g.gyro.x);
  float gy = radians(g.gyro.y);
  float gz = radians(g.gyro.z);

  // 9DoF fusion
  filter.update(gx, gy, gz,
                a.acceleration.x, a.acceleration.y, a.acceleration.z,
                mx, my, mz);

  float yaw   = filter.getYaw();    // deg
  float pitch = filter.getPitch();  // deg

  // Recenter (active LOW)
  if (digitalRead(PIN_RECENTER) == LOW) {
    recenterYaw(yaw);
    delay(200);
  }

  // Apply recenter + wrap
  yaw -= yaw_offset;
  while (yaw > 180)  yaw -= 360;
  while (yaw < -180) yaw += 360;

  // Limit to gimbal-friendly ranges
  const float YAW_MIN = -90,  YAW_MAX = 90;
  const float PIT_MIN = -45,  PIT_MAX = 45;
  yaw   = constrain(yaw,   YAW_MIN, YAW_MAX);
  pitch = constrain(pitch, PIT_MIN, PIT_MAX);

  // Smooth
  yaw_s   += ALPHA * (yaw   - yaw_s);
  pitch_s += ALPHA * (pitch - pitch_s);

  tx.yaw_deg   = yaw_s;
  tx.pitch_deg = pitch_s;

  // Send to RX
  esp_now_send(PEER_MAC, reinterpret_cast<uint8_t*>(&tx), sizeof(tx));

  // ~100 Hz
  delay(5);
}
