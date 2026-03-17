# ESP32 BLE IMU Event & Data Streaming

## Overview

This project implements a **Bluetooth Low Energy (BLE) GATT server on ESP32** that:

- Sends **event metadata** (e.g., lift, tilt, shake)
- Streams **2 seconds of IMU data (4800 bytes)**
- Uses **BLE notifications for real-time transfer**
- Uses **dummy sine wave data** (can be replaced with real IMU sensor data)


## Features

- Custom **128-bit BLE service**
- Two BLE characteristics:
  - **Event Notification**
  - **IMU Data Streaming**
- **Sequence number** for packet ordering
- Optimized using **MTU = 247**
- Button-triggered transmission (GPIO 0)
- Smooth dummy IMU data using **sine wave**


## GATT Structure

### Service
- Custom 128-bit UUID

### 1. Event Characteristic

**Properties:** Notify  

**Payload (6 bytes):**

| Field        | Size |
|-------------|------|
| Event Type   | 1 byte |
| Timestamp    | 4 bytes |
| Confidence   | 1 byte |

### 2. IMU Data Characteristic

**Properties:** Notify  

- Total data per event: **4800 bytes**
- Sent as multiple BLE notification packets


## Data Calculation


- **Total IMU Data Size**  
  200 Hz × 12 bytes × 2 sec = 4800 bytes

- **Maximum BLE Payload per Packet**  
  MTU = 247 bytes  
  ATT Header = 3 bytes  
  Available payload = 244 bytes

- **Actual Data per Packet**  
  Sequence number = 2 bytes  
  IMU data per packet = 242 bytes

## Packet Structure

Each BLE notification packet:

[ Sequence Number (2 bytes) | IMU Data (up to 242 bytes) ]

## Total Packets Required

Total packets ≈ ceil(4800 / 242) = 20 packets


## How It Works

1. ESP32 starts BLE advertising
2. Mobile device connects (use **nRF Connect**)
3. Enable notifications for:
   - Event characteristic
   - IMU characteristic
4. Press button (GPIO 0)
5. ESP32 performs:
   - Generates random event type
   - Sends event metadata
   - Generates IMU data
   - Splits into ~20 packets
   - Sends via BLE notifications


## Dummy IMU Data

IMU data is simulated using a sine wave:

```c
float val = sinf(i * freq);
int scaled = (int)((val + 1.0f) * 127.5f);
