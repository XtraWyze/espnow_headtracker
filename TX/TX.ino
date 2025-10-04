// ESP32-C3 SuperMini — HEAD UNIT (TX)
// MPU6050 + HMC5883L -> Madgwick -> send yaw/pitch via ESP-NOW

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <MadgwickAHRS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>

#define I2C_SDA   5
#define I2C_SCL   6
#define PIN_RECENTER 0   // momentary button to GND (optional)

Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag(12345);
Madgwick filter;

struct Packet {
  float yaw_deg;   // -180..+180
  float pitch_deg; // -90..+90
} tx;

// === PUT YOUR RECEIVER'S MAC HERE (after you flash RX and read it) ===
uint8_t PEER_MAC[6] = { 0x98, 0x3D, 0xAE, 0x51, 0xB3, 0x54 }; // <- EDIT ME

float yaw_offset = 0.0f;
float yaw_s = 0, pitch_s = 0;
const float ALPHA = 0.18f;  // smoothing 0..1

unsigned long lastMicros = 0;

void onSend(const uint8_t* mac_addr, esp_now_send_status_t status) {
  // Optional: debug send status
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void recenterYaw(float currentYaw) {
  yaw_offset += currentYaw;
}

void setup() {
  pinMode(PIN_RECENTER, INPUT_PULLUP);
  Serial.begin(115200);
  delay(100);

  // I2C + sensors
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(50);
  if (!mpu.begin()) { Serial.println("MPU6050 not found!"); while (1) delay(10); }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  if (!mag.begin()) { Serial.println("HMC5883L not found!"); while (1) delay(10); }

  // Wi-Fi STA for ESP-NOW
  WiFi.mode(WIFI_STA);
  // Optional: fix Wi-Fi channel to 1 (both ends should match)
  // esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  // ESP-NOW init
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (1) delay(10);
  }
  esp_now_register_send_cb(onSend);

  // Add peer (unicast)
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = 0; // 0 = current Wi-Fi channel
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (1) delay(10);
  }

  // Madgwick
  filter.begin(100.0f);
  lastMicros = micros();

  Serial.println("TX ready (ESP-NOW)");
}

void loop() {
  // Read sensors
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  sensors_event_t m;
  mag.getEvent(&m);

  // dt for filter
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
                m.magnetic.x, m.magnetic.y, m.magnetic.z);

  float yaw   = filter.getYaw();
  float pitch = filter.getPitch();

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

  // Send (unicast to peer)
  esp_now_send(PEER_MAC, (uint8_t*)&tx, sizeof(tx));

  // ~100 Hz
  delay(5);
}
