// === TX: ESP32-C3 SuperMini + MPU6050 -> ESP-NOW pan/tilt ===
// SDA=5, SCL=6, Recenter button on GPIO0
// Boosted range (max TX power, 802.11b mode)

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Wire.h>

#define WIFI_CHANNEL        1
#define I2C_SDA_PIN         5
#define I2C_SCL_PIN         6
#define I2C_FREQ_HZ         400000
#define RECENTER_BTN_PIN    0     // momentary to GND (active LOW)

#define ALPHA               0.98f
#define PITCH_LIMIT_DEG     45.0f
#define YAW_LIMIT_DEG       90.0f
#define PITCH_CENTER_DEG    0.0f
#define YAW_CENTER_DEG      0.0f
#define PITCH_SIGN          +1
#define YAW_SIGN            +1
#define TX_RATE_HZ          100
#define SERIAL_BAUD         115200

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

// ==== ESP-NOW ====
void initESPNOW(){
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if(esp_now_init()!=ESP_OK){
    Serial.println("ESP-NOW init failed");
    while(1) delay(10);
  }
  esp_now_peer_info_t peer{};
  memset(&peer,0,sizeof(peer));
  for(int i=0;i<6;i++) peer.peer_addr[i]=0xFF;
  peer.ifidx=WIFI_IF_STA;
  peer.channel=WIFI_CHANNEL;
  peer.encrypt=false;
  esp_now_add_peer(&peer);
}

void setup(){
  Serial.begin(SERIAL_BAUD);
  delay(200);
  pinMode(RECENTER_BTN_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  if(!mpuInit()){Serial.println("MPU init failed!"); while(1) delay(10);}
  Serial.println("Calibrating gyro..."); calibrateGyro(500);

  initESPNOW();
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

  uint8_t bcast[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  esp_now_send(bcast,(uint8_t*)&pkt,sizeof(pkt));

  static uint32_t dbg_t=0;
  if(millis()-dbg_t>50){
    dbg_t=millis();
    Serial.printf("seq:%lu yaw:%d pitch:%d | off:%.1f\n",
      (unsigned long)pkt.seq, pkt.yaw_deg, pkt.pitch_deg, yaw_offset);
  }

  delay(1000/TX_RATE_HZ);
}
