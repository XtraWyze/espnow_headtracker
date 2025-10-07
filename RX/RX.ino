// === RX: ESP32-C3 SuperMini -> ESP-NOW -> PAN/TILT servos ===
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

#define WIFI_CHANNEL        1
#define SERVO_PAN_PIN       4
#define SERVO_TILT_PIN      5
#define SERVO_MIN_US        1000
#define SERVO_MAX_US        2000
#define SERVO_FREQ_HZ       50
#define PAN_INVERT          0
#define TILT_INVERT         0
#define SMOOTH_ALPHA        0.35f
#define FAILSAFE_MS         250
#define LED_PIN             2

typedef struct __attribute__((packed)) {
  uint32_t seq;
  int16_t yaw_deg;
  int16_t pitch_deg;
} HeadPkt;

Servo servoPan, servoTilt;
volatile unsigned long lastRxMs = 0;
volatile HeadPkt lastPkt{};          // keep as volatile; write via memcpy
float filtPanDeg = 0.0f, filtTiltDeg = 0.0f;

static inline int degToServoAngle(float deg, bool invert) {
  if (deg > 90) deg = 90;
  if (deg < -90) deg = -90;
  if (invert) deg = -deg;
  float angle = (deg + 90.0f);
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  return (int)lroundf(angle);
}

// NEW callback signature on IDF v5.x:
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(HeadPkt)) return;

  // Copy into volatile safely
  HeadPkt pkt;
  memcpy(&pkt, data, sizeof(pkt));
  memcpy((void*)&lastPkt, &pkt, sizeof(pkt));   // write to volatile via memcpy

  lastRxMs = millis();

  // (Optional) If you want the sender MAC, it’s in info->src_addr (6 bytes)
  // const uint8_t* mac = info->src_addr;
}

void centerServos() {
  servoPan.writeMicroseconds((SERVO_MIN_US + SERVO_MAX_US)/2);
  servoTilt.writeMicroseconds((SERVO_MIN_US + SERVO_MAX_US)/2);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) delay(10);
  }

  // Register the NEW-style callback
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
    digitalWrite(LED_PIN, (now/250)%2);   // blink when link lost
    centerServos();
    delay(10);
    return;
  } else {
    digitalWrite(LED_PIN, HIGH);          // solid when link good
  }

  // Read the latest volatile packet into a local copy once per loop
  HeadPkt pkt;
  memcpy(&pkt, (const void*)&lastPkt, sizeof(pkt));

  float targetPanDeg  = (float)pkt.yaw_deg;
  float targetTiltDeg = (float)pkt.pitch_deg;

  filtPanDeg  = SMOOTH_ALPHA * targetPanDeg  + (1.0f - SMOOTH_ALPHA) * filtPanDeg;
  filtTiltDeg = SMOOTH_ALPHA * targetTiltDeg + (1.0f - SMOOTH_ALPHA) * filtTiltDeg;

  int panAngle  = degToServoAngle(filtPanDeg,  PAN_INVERT);
  int tiltAngle = degToServoAngle(filtTiltDeg, TILT_INVERT);

  servoPan.write(panAngle);
  servoTilt.write(tiltAngle);

  static uint32_t t = 0;
  if (now - t > 50) {
    t = now;
    Serial.printf("seq:%lu yaw:%d pitch:%d | pan:%d tilt:%d\n",
      (unsigned long)pkt.seq, (int)pkt.yaw_deg, (int)pkt.pitch_deg, panAngle, tiltAngle);
  }

  delay(5);
}
