// === RX: ESP32-C3 SuperMini -> Servos (pan/tilt) via RYLR998 LoRa ===
// RYLR998 connected to Serial1: TX=21, RX=20

#include <Arduino.h>
#include <ESP32Servo.h>

// Servo pins
#define SERVO_PAN_PIN 4
#define SERVO_TILT_PIN 5
#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000
#define SERVO_FREQ_HZ 50
#define PAN_INVERT 0
#define TILT_INVERT 0
#define SMOOTH_ALPHA 0.35f
#define FAILSAFE_MS 500  // Increased for LoRa latency
#define LED_PIN 2

// RYLR998 Serial pins
#define LORA_TX_PIN         7
#define LORA_RX_PIN         10
#define LORA_BAUD           115200

// RYLR998 settings
#define LORA_ADDRESS        0     // RX address (broadcast)
#define LORA_NETWORK_ID     18    // Must match TX
#define LORA_BAND           915000000   // Frequency in Hz: 433000000, 470000000, 868000000, 915000000
#define LORA_PARAMETER      "7,7,1,4"  // SpreadFactor,Bandwidth,CodingRate,Preamble (match current config)

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

void centerServos() {
  int mid = (SERVO_MIN_US + SERVO_MAX_US) / 2;
  servoPan.writeMicroseconds(mid);
  servoTilt.writeMicroseconds(mid);
}

// ==== RYLR998 LoRa helpers ====
String sendATCommand(String cmd, unsigned long timeout = 1000) {
  Serial1.println(cmd);
  unsigned long start = millis();
  String response = "";
  
  while (millis() - start < timeout) {
    if (Serial1.available()) {
      char c = Serial1.read();
      response += c;
      if (response.endsWith("\r\n") || response.endsWith("\n")) {
        break;
      }
    }
  }
  response.trim();
  return response;
}

bool initRYLR998() {
  Serial1.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  delay(100);
  
  // Test communication
  String resp = sendATCommand("AT");
  Serial.print("AT test: "); Serial.println(resp);
  if (!resp.startsWith("+OK")) return false;
  
  // Set address
  resp = sendATCommand("AT+ADDRESS=" + String(LORA_ADDRESS));
  Serial.print("Set address: "); Serial.println(resp);
  if (!resp.startsWith("+OK")) return false;
  
  // Set network ID
  resp = sendATCommand("AT+NETWORKID=" + String(LORA_NETWORK_ID));
  Serial.print("Set network ID: "); Serial.println(resp);
  if (!resp.startsWith("+OK")) return false;
  
  // Set band (frequency) - try both formats
  resp = sendATCommand("AT+BAND=" + String(LORA_BAND));
  Serial.print("Set band: "); Serial.println(resp);
  // If ERR=4, module may already be on correct band or doesn't support this command
  
  // Set parameters (SF, BW, CR, Preamble)
  resp = sendATCommand("AT+PARAMETER=" + String(LORA_PARAMETER), 2000);
  Serial.print("Set parameters: "); Serial.println(resp);
  // If ERR=4, try reading current parameters
  if (!resp.startsWith("+OK")) {
    Serial.println("Parameter set failed, reading current config...");
    resp = sendATCommand("AT+PARAMETER?");
    Serial.print("Current params: "); Serial.println(resp);
  }
  
  return true;
}

// Parse incoming LoRa data: +RCV=<Address>,<Length>,<Data>,<RSSI>,<SNR>
void parseLoRaData(String line) {
  if (!line.startsWith("+RCV=")) return;
  
  // Remove "+RCV="
  line = line.substring(5);
  
  // Split by comma
  int idx1 = line.indexOf(',');
  int idx2 = line.indexOf(',', idx1 + 1);
  int idx3 = line.indexOf(',', idx2 + 1);
  int idx4 = line.indexOf(',', idx3 + 1);
  
  if (idx1 < 0 || idx2 < 0 || idx3 < 0) return;
  
  // String addr = line.substring(0, idx1);
  String lenStr = line.substring(idx1 + 1, idx2);
  String hexData = line.substring(idx2 + 1, idx3);
  
  // Length is hex string length, convert to byte count
  int hexLen = lenStr.toInt();
  int byteLen = hexLen / 2;
  
  // Debug print
  Serial.printf("Parsing: hexLen=%d, byteLen=%d, expected=%d\n", hexLen, byteLen, sizeof(HeadPkt));
  
  if (byteLen < sizeof(HeadPkt)) {
    Serial.println("Data too short!");
    return;
  }
  
  // Convert hex string to binary
  uint8_t data[sizeof(HeadPkt)];
  for (int i = 0; i < sizeof(HeadPkt) && i * 2 < hexData.length(); i++) {
    String byteStr = hexData.substring(i * 2, i * 2 + 2);
    data[i] = (uint8_t)strtol(byteStr.c_str(), NULL, 16);
  }
  
  // Copy to packet
  HeadPkt pkt;
  memcpy(&pkt, data, sizeof(pkt));
  memcpy((void*)&lastPkt, &pkt, sizeof(pkt));
  lastRxMs = millis();
  
  Serial.printf("Decoded: seq=%lu, yaw=%d, pitch=%d\n", 
    (unsigned long)pkt.seq, pkt.yaw_deg, pkt.pitch_deg);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("Initializing RYLR998...");
  if (!initRYLR998()) {
    Serial.println("RYLR998 init failed!");
    while(1) delay(10);
  }
  Serial.println("RYLR998 ready!");

  servoPan.setPeriodHertz(SERVO_FREQ_HZ);
  servoTilt.setPeriodHertz(SERVO_FREQ_HZ);
  servoPan.attach(SERVO_PAN_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoTilt.attach(SERVO_TILT_PIN, SERVO_MIN_US, SERVO_MAX_US);

  centerServos();
  lastRxMs = millis();
  Serial.println("RX ready (listening for LoRa data)...");
}

void loop() {
  // Check for incoming LoRa data
  static String rxBuffer = "";
  while (Serial1.available()) {
    char c = Serial1.read();
    rxBuffer += c;
    
    if (c == '\n') {
      rxBuffer.trim();
      if (rxBuffer.length() > 0) {
        // Debug: print raw received data
        Serial.print("LoRa RX: ");
        Serial.println(rxBuffer);
        parseLoRaData(rxBuffer);
      }
      rxBuffer = "";
    }
  }

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
