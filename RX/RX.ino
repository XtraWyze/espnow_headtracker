// ESP32-C3 SuperMini — BASE UNIT (RX)
// ESP-NOW receiver -> Pan/Tilt servos using ESP32Servo (works on ESP32/C3/S2/S3)

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

#define SERVO_PAN_PIN   4   // If Serial conflicts, enable USB CDC On Boot or move pins
#define SERVO_TILT_PIN  5

struct Packet {
  float yaw_deg;
  float pitch_deg;
};

Servo servoPan, servoTilt;
volatile Packet latest{};
volatile uint32_t lastMs = 0;

static inline int mapf_to_int(float v, float in_min, float in_max, int out_min, int out_max) {
  if (v < in_min) v = in_min;
  if (v > in_max) v = in_max;
  return (int)(out_min + (v - in_min) * (out_max - out_min) / (in_max - in_min));
}

// NEW signature for ESP-NOW recv callback in recent ESP-IDF/Arduino-ESP32
void onRecv(const esp_now_recv_info* info, const uint8_t* incomingData, int len) {
  if (len == sizeof(Packet)) {
    memcpy((void*)&latest, incomingData, sizeof(Packet)); // copy into volatile
    lastMs = millis();
  }
}

void setup() {
  // Tip: Tools -> USB CDC On Boot: Enabled (so Serial uses USB, freeing GPIO1)
  Serial.begin(115200);
  delay(100);

  // (Optional) allocate all 4 PWM timers up-front to avoid conflicts
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Set 50 Hz per instance (or omit: default is 50 Hz)
  servoPan.setPeriodHertz(50);
  servoTilt.setPeriodHertz(50);

  // Attach with safe pulse limits
  servoPan.attach(SERVO_PAN_PIN, 1000, 2000);
  servoTilt.attach(SERVO_TILT_PIN, 1000, 2000);
  servoPan.write(90);
  servoTilt.write(90);

  WiFi.mode(WIFI_STA);
  Serial.print("Receiver MAC: "); Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (true) delay(10);
  }
  esp_now_register_recv_cb(onRecv);

  Serial.println("RX ready (ESP-NOW + ESP32Servo)");
}

void loop() {
  // Copy from volatile safely into a local (non-volatile) Packet
  Packet p;
  memcpy(&p, (const void*)&latest, sizeof(Packet));

  // Map: yaw -90..+90 -> pan 0..180
  //      pitch -45..+45 -> tilt 30..150
  int panDeg  = mapf_to_int(p.yaw_deg,   -90.0f, +90.0f, 0,   180);
  int tiltDeg = mapf_to_int(p.pitch_deg, -45.0f, +45.0f, 30,  150);

  servoPan.write(panDeg);
  servoTilt.write(tiltDeg);

  // Failsafe if no packets for 700 ms
  if (millis() - lastMs > 700) {
    servoPan.write(90);
    servoTilt.write(90);
  }

  delay(5);
}
