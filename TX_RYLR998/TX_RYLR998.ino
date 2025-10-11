// === TX: ESP32-C3 SuperMini + MPU6050 -> RYLR998 LoRa pan/tilt ===
// SDA=5, SCL=6, Recenter button on GPIO0
// RYLR998 connected to Serial1: TX=21, RX=20

#include <Arduino.h>
#include <Wire.h>

// Hardware pins
#define I2C_SDA_PIN         5
#define I2C_SCL_PIN         6
#define I2C_FREQ_HZ         400000
#define RECENTER_BTN_PIN    0     // momentary to GND (active LOW)

// RYLR998 Serial pins (use Serial1 on ESP32-C3)
#define LORA_TX_PIN         10
#define LORA_RX_PIN         7
#define LORA_BAUD           115200

// MPU6050 settings
#define ALPHA               0.98f
#define PITCH_LIMIT_DEG     45.0f
#define YAW_LIMIT_DEG       90.0f
#define PITCH_CENTER_DEG    0.0f
#define YAW_CENTER_DEG      0.0f
#define PITCH_SIGN          +1
#define YAW_SIGN            +1
#define TX_RATE_HZ          10  // Reduced for LoRa transmission time (SF7 ~100ms per packet)
#define SERIAL_BAUD         115200

// RYLR998 settings
#define LORA_ADDRESS        1     // TX address
#define LORA_NETWORK_ID     18    // Must match on TX and RX
#define LORA_BAND           915000000   // Frequency in Hz: 433000000, 470000000, 868000000, 915000000
#define LORA_PARAMETER      "7,7,1,4"  // SpreadFactor,Bandwidth,CodingRate,Preamble (match current config)

// MPU6050 registers
#define MPU6050_ADDR        0x68
#define REG_PWR_MGMT_1      0x6B
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B

static const float ACC_SCALE = 16384.0f;
static const float GYR_SCALE = 131.0f;

typedef struct __attribute__((packed)) {
  uint32_t seq;
  int16_t yaw_deg;
  int16_t pitch_deg;
} HeadPkt;

volatile uint32_t seq_no = 0;
float pitch_deg = 0, yaw_deg = 0, yaw_offset = 0;
float gx_bias = 0, gy_bias = 0, gz_bias = 0;
unsigned long last_us;
bool btnPrev = true;
unsigned long btnDebounceMs = 0;
const unsigned long BTN_DEBOUNCE = 200;

// ==== I2C helpers ====
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuReadBytes(uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(MPU6050_ADDR, (uint8_t)n);
  for (size_t i=0; i<n && Wire.available(); ++i) buf[i] = Wire.read();
  return true;
}

bool mpuInit() {
  if (!mpuWrite(REG_PWR_MGMT_1, 0x00)) return false;
  delay(50);
  mpuWrite(REG_SMPLRT_DIV, 0x04);
  mpuWrite(REG_CONFIG, 0x03);
  mpuWrite(REG_GYRO_CONFIG, 0x00);
  mpuWrite(REG_ACCEL_CONFIG, 0x00);
  delay(50);
  return true;
}

bool mpuReadRaw(int16_t &ax,int16_t &ay,int16_t &az,int16_t &gx,int16_t &gy,int16_t &gz){
  uint8_t buf[14];
  if (!mpuReadBytes(REG_ACCEL_XOUT_H, buf, 14)) return false;
  ax=(int16_t)((buf[0]<<8)|buf[1]);
  ay=(int16_t)((buf[2]<<8)|buf[3]);
  az=(int16_t)((buf[4]<<8)|buf[5]);
  gx=(int16_t)((buf[8]<<8)|buf[9]);
  gy=(int16_t)((buf[10]<<8)|buf[11]);
  gz=(int16_t)((buf[12]<<8)|buf[13]);
  return true;
}

void calibrateGyro(size_t samples=500){
  long gx_sum=0,gy_sum=0,gz_sum=0;
  for(size_t i=0;i<samples;i++){
    int16_t ax,ay,az,gx,gy,gz;
    if(mpuReadRaw(ax,ay,az,gx,gy,gz)){gx_sum+=gx;gy_sum+=gy;gz_sum+=gz;}
    delay(2);
  }
  gx_bias=(float)gx_sum/samples;
  gy_bias=(float)gy_sum/samples;
  gz_bias=(float)gz_sum/samples;
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

void sendLoRaData(uint8_t destAddr, const uint8_t* data, size_t len) {
  // Convert binary data to hex string
  String hexData = "";
  for (size_t i = 0; i < len; i++) {
    char hex[3];
    sprintf(hex, "%02X", data[i]);
    hexData += hex;
  }
  
  // AT+SEND=<Address>,<Length>,<Data>
  // Length should be the hex string length (2 chars per byte)
  String cmd = "AT+SEND=" + String(destAddr) + "," + String(hexData.length()) + "," + hexData;
  Serial1.println(cmd);
  
  // Debug: Read response from module
  static uint32_t debug_count = 0;
  if (debug_count++ % 20 == 0) {  // Print every 20th packet
    delay(10);  // Wait for response
    while (Serial1.available()) {
      String resp = Serial1.readStringUntil('\n');
      resp.trim();
      Serial.print("LoRa TX resp: ");
      Serial.println(resp);
    }
  }
}

void setup(){
  Serial.begin(SERIAL_BAUD);
  delay(200);
  pinMode(RECENTER_BTN_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  if(!mpuInit()){Serial.println("MPU init failed!"); while(1) delay(10);}
  Serial.println("Calibrating gyro..."); calibrateGyro(500);

  Serial.println("Initializing RYLR998...");
  if (!initRYLR998()) {
    Serial.println("RYLR998 init failed!");
    while(1) delay(10);
  }
  Serial.println("RYLR998 ready!");

  last_us = micros();
}

void loop(){
  unsigned long now_us=micros();
  float dt=(now_us-last_us)*1e-6f;
  if(dt<=0) dt=1e-3f;
  last_us=now_us;

  int16_t ax,ay,az,gx,gy,gz;
  if(!mpuReadRaw(ax,ay,az,gx,gy,gz)) return;

  float ax_g=ax/ACC_SCALE, ay_g=ay/ACC_SCALE, az_g=az/ACC_SCALE;
  float gx_dps=(gx-gx_bias)/GYR_SCALE, gy_dps=(gy-gy_bias)/GYR_SCALE, gz_dps=(gz-gz_bias)/GYR_SCALE;
  float acc_pitch=atan2f(-ax_g, sqrtf(ay_g*ay_g+az_g*az_g))*180.0f/PI;

  float pitch_pred=pitch_deg+(gy_dps)*dt;
  float yaw_pred=yaw_deg+(gz_dps)*dt;

  pitch_deg=ALPHA*pitch_pred+(1.0f-ALPHA)*acc_pitch;
  yaw_deg=yaw_pred - yaw_offset;

  // recenter button
  bool btnNow=(digitalRead(RECENTER_BTN_PIN)==LOW);
  unsigned long ms=millis();
  if(btnNow!=btnPrev && (ms-btnDebounceMs)>BTN_DEBOUNCE){
    btnDebounceMs=ms;
    if(btnNow){ yaw_offset=yaw_pred; Serial.println("Yaw recentered"); }
    btnPrev=btnNow;
  }

  float out_pitch=(PITCH_SIGN)*(pitch_deg+PITCH_CENTER_DEG);
  float out_yaw=(YAW_SIGN)*(yaw_deg+YAW_CENTER_DEG);
  out_pitch = constrain(out_pitch, -PITCH_LIMIT_DEG, PITCH_LIMIT_DEG);
  out_yaw   = constrain(out_yaw, -YAW_LIMIT_DEG, YAW_LIMIT_DEG);

  HeadPkt pkt;
  pkt.seq=seq_no++;
  pkt.yaw_deg=(int16_t)roundf(out_yaw);
  pkt.pitch_deg=(int16_t)roundf(out_pitch);

  // Send via LoRa to broadcast address 0
  sendLoRaData(0, (uint8_t*)&pkt, sizeof(pkt));

  static uint32_t dbg_t=0;
  if(millis()-dbg_t>50){
    dbg_t=millis();
    Serial.printf("seq:%lu yaw:%d pitch:%d | off:%.1f\n",
      (unsigned long)pkt.seq, pkt.yaw_deg, pkt.pitch_deg, yaw_offset);
  }

  delay(1000/TX_RATE_HZ);
}
