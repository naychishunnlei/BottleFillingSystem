/*
 * Bottle Filling System — RECEIVER ESP32 (IoT Gateway)
 * Cyber-Physical Systems & Industry 4.0 — Scenario A
 *
 * Role in the CPS architecture:
 *   Controller ESP32 ──CAN bus──► Receiver ESP32 ──WiFi/HTTP──► Backend ──► Dashboard
 *                    ◄────────────────────────────────────────── (control commands)
 *
 * This ESP32 acts as the IoT gateway:
 *   1. Receives real-time system state from Controller via CAN bus (TWAI)
 *   2. Forwards state to backend via HTTP POST every 1 s (digital twin sync)
 *   3. Polls backend every 500 ms for dashboard control commands
 *   4. Sends control commands back to Controller via CAN bus
 *
 * WIRING — CAN Transceiver (SN65HVD230 or TJA1050)
 *   ESP32 GPIO 21 → TX pin of transceiver
 *   ESP32 GPIO 22 → RX pin of transceiver
 *   Transceiver CANH / CANL → matched to Controller transceiver
 *   120Ω termination resistor across CANH–CANL at each end of the bus
 *
 * CAN MESSAGE IDs
 *   0x100  receive  — 2-byte packed boolean flags from Controller
 *   0x101  receive  — 8-byte counters from Controller
 *   0x200  transmit — 1-byte control command to Controller
 *              0x01 = start | 0x02 = stop | 0x03 = emergency/reset
 *
 * FreeRTOS Tasks
 *   taskCANReceive  core 1, blocking    — receives state frames from Controller
 *   taskCANControl  core 1, 500 ms      — sends control commands to Controller
 *   taskComms       core 0, 1 s         — WiFi keep-alive + HTTP POST /data
 *   taskControl     core 0, 500 ms      — polls GET /control/pending from backend
 *   taskSerial      core 0, 1 s         — Serial Monitor state dump
 */

#include "driver/twai.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ─── Config ───────────────────────────────────────────────────────────────────
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* BACKEND_URL   = "http://192.168.1.100:8000";  // your PC's IP

// ─── CAN ──────────────────────────────────────────────────────────────────────
#define CAN_TX_PIN      21
#define CAN_RX_PIN      22
#define CAN_ID_FLAGS    0x100   // receive: boolean state flags
#define CAN_ID_COUNTERS 0x101   // receive: bottle counter, fault count, uptime
#define CAN_ID_CONTROL  0x200   // transmit: dashboard control command to Controller

// Control command bytes (sent in CAN_ID_CONTROL frame data[0])
#define CMD_START     0x01
#define CMD_STOP      0x02
#define CMD_EMERGENCY 0x03

// ─── Shared state ─────────────────────────────────────────────────────────────
struct SystemState {
  // Inputs — mirror of Controller sensor readings
  bool bottlePresent, bottlePositioned;

  // Internal logic bits — mirror of Controller ladder coils
  bool systemRunning, fault, emergencyStop;
  bool fillEnable, fillingComplete;
  bool runLatch, conveyorMotor, valveOn;

  // Outputs — mirror of Controller LED/actuator states
  bool greenLED, yellowLED, redLED, buzzerOn;

  // Counters — software counter (CTU) and uptime from Controller
  int  bottleCounter, faultCount, uptime;
};

SystemState       g_state    = {};
SemaphoreHandle_t g_mutex;

// Pending control command polled from backend, consumed by taskCANControl
volatile uint8_t  g_pendingCmd = 0x00;

// ─── CAN frame unpacking ──────────────────────────────────────────────────────
// Mirrors the bit-packing in controller.ino packFlags0 / packFlags1
void unpackFlags(uint8_t b0, uint8_t b1, SystemState& s) {
  s.systemRunning    = (b0 >> 7) & 1;
  s.fault            = (b0 >> 6) & 1;
  s.emergencyStop    = (b0 >> 5) & 1;
  s.bottlePresent    = (b0 >> 4) & 1;
  s.bottlePositioned = (b0 >> 3) & 1;
  s.fillEnable       = (b0 >> 2) & 1;
  s.fillingComplete  = (b0 >> 1) & 1;
  s.runLatch         = (b0 >> 0) & 1;
  s.conveyorMotor    = (b1 >> 7) & 1;
  s.valveOn          = (b1 >> 6) & 1;
  s.greenLED         = (b1 >> 5) & 1;
  s.yellowLED        = (b1 >> 4) & 1;
  s.redLED           = (b1 >> 3) & 1;
  s.buzzerOn         = (b1 >> 2) & 1;
}

void unpackCounters(uint8_t* d, SystemState& s) {
  s.bottleCounter = ((uint16_t)d[0] << 8) | d[1];
  s.faultCount    = ((uint16_t)d[2] << 8) | d[3];
  s.uptime        = ((uint32_t)d[4] << 24) | ((uint32_t)d[5] << 16) |
                    ((uint32_t)d[6] <<  8) |  d[7];
}

// ═════════════════════════════════════════════════════════════════════════════
// TASK: taskCANReceive  (core 1, blocking on twai_receive)
// Waits for CAN frames from Controller and unpacks into g_state.
// Frame 0x100 → boolean flags | Frame 0x101 → counters
// ═════════════════════════════════════════════════════════════════════════════
void taskCANReceive(void* pv) {
  Serial.println("[taskCANReceive] started");

  for (;;) {
    twai_message_t msg;
    // Block until a frame arrives (100 ms timeout to stay responsive)
    if (twai_receive(&msg, pdMS_TO_TICKS(100)) != ESP_OK) continue;

    xSemaphoreTake(g_mutex, portMAX_DELAY);

    if (msg.identifier == CAN_ID_FLAGS && msg.data_length_code >= 2) {
      unpackFlags(msg.data[0], msg.data[1], g_state);
      Serial.printf("[CAN RX] 0x100 flags: 0x%02X 0x%02X\n", msg.data[0], msg.data[1]);

    } else if (msg.identifier == CAN_ID_COUNTERS && msg.data_length_code == 8) {
      unpackCounters(msg.data, g_state);
      Serial.printf("[CAN RX] 0x101 bottles:%d faults:%d uptime:%ds\n",
        g_state.bottleCounter, g_state.faultCount, g_state.uptime);
    }

    xSemaphoreGive(g_mutex);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TASK: taskCANControl  (core 1, 500 ms)
// Forwards pending control commands from dashboard to Controller via CAN.
// Sends CAN frame 0x200 with one command byte:
//   0x01 = start | 0x02 = stop | 0x03 = emergency/reset
// ═════════════════════════════════════════════════════════════════════════════
void taskCANControl(void* pv) {
  Serial.println("[taskCANControl] started");

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(500));

    uint8_t cmd = g_pendingCmd;
    if (cmd == 0x00) continue;  // nothing to send
    g_pendingCmd = 0x00;

    twai_message_t msg = {};
    msg.identifier       = CAN_ID_CONTROL;
    msg.data_length_code = 1;
    msg.data[0]          = cmd;

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(10));
    if (err == ESP_OK) {
      const char* name = (cmd == CMD_START) ? "START" :
                         (cmd == CMD_STOP)  ? "STOP"  : "EMERGENCY";
      Serial.printf("[CAN TX] 0x200 command: %s (0x%02X)\n", name, cmd);
    } else {
      Serial.printf("[CAN TX] 0x200 transmit error: %d\n", err);
    }
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TASK: taskComms  (core 0, 1 s)
// Industry 4.0 data pipeline: sends system state snapshot to backend (InfluxDB).
// Maintains WiFi connection with auto-reconnect.
// ═════════════════════════════════════════════════════════════════════════════
void taskComms(void* pv) {
  // Connect on startup
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.printf("\n[WiFi] Connected — IP: %s\n", WiFi.localIP().toString().c_str());

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Disconnected, reconnecting...");
      WiFi.reconnect();
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }

    // Take a snapshot so we hold the mutex for as short as possible
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    SystemState s = g_state;
    xSemaphoreGive(g_mutex);

    // Build JSON payload matching the SystemDataIn schema on the backend
    StaticJsonDocument<512> doc;
    doc["systemRunning"]    = s.systemRunning;
    doc["fault"]            = s.fault;
    doc["emergencyStop"]    = s.emergencyStop;
    doc["bottlePresent"]    = s.bottlePresent;
    doc["bottlePositioned"] = s.bottlePositioned;
    doc["fillEnable"]       = s.fillEnable;
    doc["fillingComplete"]  = s.fillingComplete;
    doc["runLatch"]         = s.runLatch;
    doc["conveyorMotor"]    = s.conveyorMotor;
    doc["valveOn"]          = s.valveOn;
    doc["greenLED"]         = s.greenLED;
    doc["yellowLED"]        = s.yellowLED;
    doc["redLED"]           = s.redLED;
    doc["buzzer"]           = s.buzzerOn;
    doc["bottleCounter"]    = s.bottleCounter;
    doc["faultCount"]       = s.faultCount;
    doc["uptime"]           = s.uptime;

    String body;
    serializeJson(doc, body);

    HTTPClient http;
    http.begin(String(BACKEND_URL) + "/data");
    http.addHeader("Content-Type", "application/json");
    int code = http.POST(body);
    Serial.printf("[HTTP] POST /data → %d\n", code);
    http.end();
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TASK: taskControl  (core 0, 500 ms)
// Polls backend GET /control/pending for dashboard control commands.
// If an action is queued, maps it to a CAN command byte for taskCANControl.
// Enables remote control of the physical system from the digital twin dashboard.
// ═════════════════════════════════════════════════════════════════════════════
void taskControl(void* pv) {
  Serial.println("[taskControl] started");

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(500));

    if (WiFi.status() != WL_CONNECTED) continue;

    HTTPClient http;
    http.begin(String(BACKEND_URL) + "/control/pending");
    int code = http.GET();

    if (code == 200) {
      String payload = http.getString();
      StaticJsonDocument<128> doc;
      if (deserializeJson(doc, payload) == DeserializationError::Ok) {
        String action = doc["action"].as<String>();

        if (action == "start") {
          g_pendingCmd = CMD_START;
          Serial.println("[CTRL] Queued: START");
        } else if (action == "stop") {
          g_pendingCmd = CMD_STOP;
          Serial.println("[CTRL] Queued: STOP");
        } else if (action == "reset" || action == "emergency") {
          g_pendingCmd = CMD_EMERGENCY;
          Serial.println("[CTRL] Queued: EMERGENCY/RESET");
        }
        // empty string means no pending command — do nothing
      }
    }
    http.end();
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TASK: taskSerial  (core 0, 1 s)
// Prints a state snapshot to Serial Monitor for debugging.
// ═════════════════════════════════════════════════════════════════════════════
void taskSerial(void* pv) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    SystemState s = g_state;
    xSemaphoreGive(g_mutex);

    Serial.println("─────────────────────────");
    Serial.printf("  running   : %s\n", s.systemRunning ? "YES" : "NO");
    Serial.printf("  fault     : %s\n", s.fault         ? "YES" : "NO");
    Serial.printf("  emergency : %s\n", s.emergencyStop ? "YES" : "NO");
    Serial.printf("  present   : %s\n", s.bottlePresent    ? "YES" : "NO");
    Serial.printf("  positioned: %s\n", s.bottlePositioned ? "YES" : "NO");
    Serial.printf("  valve     : %s\n", s.valveOn       ? "OPEN" : "closed");
    Serial.printf("  bottles   : %d\n", s.bottleCounter);
    Serial.printf("  faults    : %d\n", s.faultCount);
    Serial.printf("  uptime    : %d s\n", s.uptime);
    Serial.println("─────────────────────────");
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// setup / loop
// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== RECEIVER ESP32 (IoT Gateway) BOOT ===");

  // Init TWAI (ESP32 built-in CAN controller) at 500 kbps
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK ||
      twai_start() != ESP_OK) {
    Serial.println("[CAN] Init FAILED — halting.");
    while (true) delay(1000);
  }
  Serial.println("[CAN] TWAI ready at 500 kbps");

  g_mutex = xSemaphoreCreateMutex();
  configASSERT(g_mutex);

  //                            function        name          stack   arg  prio handle core
  xTaskCreatePinnedToCore(taskCANReceive,  "CANReceive",  4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskCANControl,  "CANControl",  2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskComms,       "Comms",       8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskControl,     "Control",     4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskSerial,      "Serial",      4096, NULL, 1, NULL, 0);

  Serial.println("=== All tasks started. Waiting for CAN frames. ===\n");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
