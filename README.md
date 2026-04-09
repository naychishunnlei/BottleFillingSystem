# BottleFillingSystem
# ESP32 Firmware — Bottle Filling System

Cyber-Physical Systems & Industry 4.0 — Scenario A

---

## Overview

Two ESP32 boards communicate over a CAN bus. The **Controller** runs all the
filling logic and drives the physical hardware. The **Receiver** acts as an IoT
gateway — it forwards system state to the backend and relays dashboard control
commands back to the Controller.

```
Sensors/Switches
      │
      ▼
┌─────────────────┐   CAN bus (0x100, 0x101)   ┌─────────────────┐
│  Controller     │ ─────────────────────────► │  Receiver       │
│  ESP32          │ ◄───────────────────────── │  ESP32          │
│                 │   CAN bus (0x200)           │                 │
│  • Ladder logic │                             │  • WiFi         │
│  • FreeRTOS     │                             │  • HTTP POST    │
│  • Actuators    │                             │  • Poll /ctrl   │
└─────────────────┘                             └────────┬────────┘
                                                         │ HTTP REST
                                                         ▼
                                                    Backend (FastAPI)
                                                         │
                                                    InfluxDB / Dashboard
```

---

## Hardware

| Qty | Component | Notes |
|-----|-----------|-------|
| 2 | ESP32 Development Board | Any 38-pin variant |
| 2 | CAN Bus Transceiver | SN65HVD230 (3.3 V) recommended |
| 2 | Sensors | IR or ultrasonic |
| 3 | Push buttons | Momentary NO |
| 3 | LEDs | Green / Yellow / Red |
| 1 | Relay module | For filling valve |
| 1 | Active buzzer | 3.3 V or 5 V |
| — | 120 Ω resistors (×2) | CAN bus termination, one at each end |
| — | 220 Ω resistors | One per LED |

---

## Pin Assignment

### Controller ESP32

| GPIO | Component |
|------|-----------|
| 34 | Sensor 1 — Bottle presence |
| 35 | Sensor 2 — Bottle positioned |
| 25 | Switch 1 — Start |
| 26 | Switch 2 — Stop |
| 27 | Switch 3 — Emergency stop / Reset |
| 32 | Relay — Filling valve |
| 19 | LED Green — System running |
| 18 | LED Yellow — Filling in progress |
| 5 | LED Red — Fault / stopped |
| 17 | Buzzer — Cycle complete |
| **21** | **CAN TX → transceiver TX pin** |
| **22** | **CAN RX → transceiver RX pin** |

### Receiver ESP32

| GPIO | Component |
|------|-----------|
| **21** | **CAN TX → transceiver TX pin** |
| **22** | **CAN RX → transceiver RX pin** |

> No other hardware is wired to the Receiver — it only needs power and WiFi.

### CAN Bus Wiring

```
Controller transceiver          Receiver transceiver
  CANH ──────────────────────── CANH
  CANL ──────────────────────── CANL
  │                                │
 120Ω (across CANH–CANL)        120Ω (across CANH–CANL)
```

---

## Software Dependencies

### Arduino IDE Setup

1. Install **Arduino IDE 2.x** from [arduino.cc](https://www.arduino.cc/en/software)
2. Add the ESP32 board package:
   - Go to **File → Preferences**
   - Add this URL to *Additional Boards Manager URLs*:
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Go to **Tools → Board → Boards Manager**, search `esp32`, install **esp32 by Espressif Systems**

### Libraries

Install all via **Tools → Manage Libraries**:

| Library | Author | Version |
|---------|--------|---------|
| ArduinoJson | Benoit Blanchon | ≥ 7.x |

> **No extra CAN library needed.** Both files use `driver/twai.h` which is
> bundled with the ESP32 Arduino core (Espressif). It provides direct access to
> the ESP32's built-in TWAI (CAN 2.0B) controller.

---

## Configuration

Before uploading, edit the following in each file:

### controller/controller.ino

No WiFi needed. Only change timing constants if required:

```cpp
const TickType_t TON_FILL   = pdMS_TO_TICKS(5000);   // fill duration (ms)
const TickType_t TON_BUZZER = pdMS_TO_TICKS(500);     // buzzer pulse (ms)
const TickType_t TON_FAULT  = pdMS_TO_TICKS(15000);   // fault timeout (ms)
```

### receiver/receiver.ino

```cpp
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* BACKEND_URL   = "http://192.168.1.100:8000";  // your PC's local IP
```

Find your PC's IP: open **Command Prompt** → run `ipconfig` → look for
**IPv4 Address** under your WiFi adapter.

---

## Upload Instructions

> Upload order does not matter — both boards operate independently on power-up.

1. Open `controller/controller.ino` in Arduino IDE
2. Select board: **Tools → Board → esp32 → ESP32 Dev Module**
3. Select the correct COM port
4. Click **Upload**
5. Repeat for `receiver/receiver.ino` on the second ESP32

Open **Serial Monitor** at **115200 baud** on either board to see live output.

---

## CAN Message Format

| ID | Direction | Bytes | Content |
|----|-----------|-------|---------|
| `0x100` | Controller → Receiver | 2 | Packed boolean flags |
| `0x101` | Controller → Receiver | 8 | `bottleCounter` (u16) + `faultCount` (u16) + `uptime` (u32) |
| `0x200` | Receiver → Controller | 1 | Control command: `0x01`=start, `0x02`=stop, `0x03`=emergency |

### Flag byte layout (0x100)

```
Byte 0: [systemRunning | fault | emergencyStop | bottlePresent |
          bottlePositioned | fillEnable | fillingComplete | runLatch]

Byte 1: [conveyorMotor | valveOn | greenLED | yellowLED |
          redLED | buzzerOn | 0 | 0]
```

---

## Backend API Endpoints (used by Receiver)

| Method | Endpoint | Description |
|--------|----------|-------------|
| `POST` | `/data` | Receiver sends system state every 1 s |
| `GET` | `/control/pending` | Receiver polls for dashboard commands every 500 ms |
| `GET` | `/latest` | Dashboard reads current state |
| `GET` | `/bottle-count` | Dashboard reads production trend |
| `GET` | `/history` | Dashboard reads historical chart data |
| `POST` | `/control` | Dashboard sends start / stop / reset |

---

## FreeRTOS Task Summary

### Controller

| Task | Core | Period | Job |
|------|------|--------|-----|
| `taskInputs` | 1 | 10 ms | Read sensors, detect switch edges |
| `taskStateMachine` | 1 | 10 ms | Ladder logic — filling sequence |
| `taskOutputs` | 1 | 10 ms | Drive relay, LEDs, buzzer |
| `taskUptime` | 1 | 1 s | Increment uptime counter |
| `taskCAN` | 0 | 200 ms | Transmit 0x100 and 0x101 frames |
| `taskSerial` | 0 | 1 s | Serial Monitor dump |

### Receiver

| Task | Core | Period | Job |
|------|------|--------|-----|
| `taskCANReceive` | 1 | blocking | Receive 0x100 / 0x101 frames |
| `taskCANControl` | 1 | 500 ms | Transmit 0x200 control frame |
| `taskComms` | 0 | 1 s | POST state to backend |
| `taskControl` | 0 | 500 ms | Poll backend for commands |
| `taskSerial` | 0 | 1 s | Serial Monitor dump |
