# ESP-NOW Head Tracker - RYLR998 LoRa Version

This folder contains the RYLR998 LoRa module variant of the ESP-NOW head tracker system.

## Hardware Requirements

### Transmitter (TX)
- ESP32-C3 SuperMini
- MPU6050 IMU (I2C)
- RYLR998 LoRa Module
- Push button (recenter)

### Receiver (RX)
- ESP32-C3 SuperMini
- RYLR998 LoRa Module
- 2x Servos (pan/tilt)

## Wiring

### TX Wiring
```
MPU6050:
  SDA -> GPIO5
  SCL -> GPIO6
  VCC -> 3.3V
  GND -> GND

RYLR998:
  TX  -> GPIO10 (ESP32 RX)
  RX  -> GPIO7  (ESP32 TX)
  VCC -> 3.3V
  GND -> GND

Button:
  One side -> GPIO0
  Other side -> GND
```

### RX Wiring
```
RYLR998:
  TX  -> GPIO10 (ESP32 RX)
  RX  -> GPIO7  (ESP32 TX)
  VCC -> 3.3V
  GND -> GND

Servos:
  Pan Servo  -> GPIO4
  Tilt Servo -> GPIO5
  Both servos VCC -> 5V (external power recommended)
  Both servos GND -> GND
```

## RYLR998 Configuration

The modules are automatically configured on startup:

- **Network ID**: 18 (must match on TX and RX)
- **TX Address**: 1
- **RX Address**: 0 (broadcast)
- **Frequency**: 915 MHz (configurable for your region)
- **Parameters**: SF7, BW7, CR1, Preamble 4
- **Baud Rate**: 115200

## Key Differences from ESP-NOW Version

1. **Lower Update Rate**: 10Hz instead of 100Hz (LoRa transmission takes ~100ms per packet)
2. **Longer Failsafe**: 500ms instead of 250ms (accounts for LoRa latency)
3. **Greater Range**: LoRa can achieve several kilometers line-of-sight
4. **Different Pins**: Uses GPIO7/10 for Serial1 communication with RYLR998

## Configuration

### Frequency Bands
Adjust `LORA_BAND` for your region:
- 433000000 (433 MHz - Global)
- 470000000 (470 MHz - China)
- 868000000 (868 MHz - Europe)
- 915000000 (915 MHz - Americas, Australia)

### LoRa Parameters
The `LORA_PARAMETER` setting controls:
- Spreading Factor (7-12): Higher = longer range, slower speed
- Bandwidth (7-9): 7=125kHz, 8=250kHz, 9=500kHz
- Coding Rate (1-4): Higher = more error correction
- Preamble length (4-65535): Usually 4-8

Current setting: `"7,7,1,4"` = SF7, BW125kHz, CR4/5, Preamble 4

## Libraries Required

All libraries are built-in to the ESP32 Arduino core:
- Arduino.h
- Wire.h (for MPU6050)
- ESP32Servo.h (for RX only)

## Installation

1. Install ESP32 board support in Arduino IDE
2. Select board: **ESP32C3 Dev Module**
3. Upload TX_RYLR998.ino to transmitter
4. Upload RX_RYLR998.ino to receiver
5. Power both units and test

## Troubleshooting

### No communication between modules
- Verify both modules on same Network ID (18)
- Check wiring (TX/RX pins crossed correctly)
- Ensure both modules have stable 3.3V power
- Check if modules are same frequency variant

### Getting +ERR=17 (Busy)
- Normal to see occasional errors
- If constant, reduce TX_RATE_HZ further

### Getting +ERR=5 (Length error)
- Code should handle this automatically
- Check that firmware is up to date

### Servos jittering
- Add external 5V power supply for servos
- Adjust SMOOTH_ALPHA (lower = smoother, higher = more responsive)

## Performance

- **Update Rate**: 10 Hz
- **Latency**: ~100-150ms
- **Range**: 500m-2km depending on environment and antenna
- **RSSI/SNR**: Displayed in debug output (good signal: RSSI > -100, SNR > 0)

## Credits

Original ESP-NOW version by kolyw
RYLR998 LoRa adaptation by GitHub Copilot
