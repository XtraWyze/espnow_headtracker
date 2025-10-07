// === RX: ESP32-C3 SuperMini -> Servos (pan/tilt) via ESP-NOW ===
// Compatible with latest ESP32 Arduino core (IDF v5.x)
// Includes boosted range (max power, 11b mode)

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

#define WIFI_CHANNEL 1
#define SERVO_PAN_PIN 4
#define SERVO_TILT_PIN 5
#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000
#define SERVO_FREQ_HZ 50
#define PAN_INVERT 0
#define TILT_INVERT 0
#define SMOOTH_ALPHA 0.35f
#define FAILSAFE_MS 250
#define LED_PIN 2

typedef struct __attribute__((packed)) {
  uint32_t seq;
  int16_t yaw_deg;
  int16_t pitch_deg;
} HeadPkt;

Servo servoPan, servoTilt;
volatile unsigned long lastRxMs = 0;
volatile HeadPkt lastPkt{};
float filtPanDeg = 0, filtTiltDeg = 0;

static inline int degToServoAngle(float deg, bool invert) {
  deg = constrain(deg, -90, 90);
  if (invert) deg = -deg;
  return (int)lroundf((deg + 90.0f));
}

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(HeadPkt)) return;
  HeadPkt pkt;
  memcpy(&pkt, data, sizeof(pkt));
  memcpy((void*)&lastPkt, &pkt, sizeof(pkt));
  lastRxMs = millis();
}

void centerServos() {
  int mid = (SERVO_MIN_US + SERVO_MAX_US) / 2;
  servoPan.writeMicroseconds(mid);
  servoTilt.writeMicroseconds(mid);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) delay(10);
  }
  esp_now_register_recv_cb(onRecv);

  servoPan.setPeriodHertz(SERVO_FREQ_HZ);
  servoTilt.setPeriodHertz(SERVO_FREQ_HZ);
  servoPan.attach(SERVO_PAN_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoTilt.attach(SERVO_TILT_PIN, SERVO_MIN_US, SERVO_MAX_US);

  centerServos();
  lastRxMs = millis();
  Serial.println("RX ready (listening for broadcast)...");
}

void loop() {
  unsigned long now = millis();
  bool link = (now - lastRxMs) < FAILSAFE_MS;

  if (!link) {
    digitalWrite(LED_PIN, (now/250)%2);
    centerServos();
    delay(10);
    return;
  } else {
    digitalWrite(LED_PIN, HIGH);
  }

  HeadPkt pkt;
  memcpy(&pkt, (const void*)&lastPkt, sizeof(pkt));

  filtPanDeg  = SMOOTH_ALPHA * pkt.yaw_deg   + (1.0f - SMOOTH_ALPHA) * filtPanDeg;
  filtTiltDeg = SMOOTH_ALPHA * pkt.pitch_deg + (1.0f - SMOOTH_ALPHA) * filtTiltDeg;

  int panAngle  = degToServoAngle(filtPanDeg, PAN_INVERT);
  int tiltAngle = degToServoAngle(filtTiltDeg, TILT_INVERT);

  servoPan.write(panAngle);
  servoTilt.write(tiltAngle);

  static uint32_t t = 0;
  if (now - t > 50) {
    t = now;
    Serial.printf("seq:%lu yaw:%d pitch:%d | pan:%d tilt:%d\n",
      (unsigned long)pkt.seq, pkt.yaw_deg, pkt.pitch_deg, panAngle, tiltAngle);
  }

  delay(5);
}
