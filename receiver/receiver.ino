/*
 * Bottle Filling System — RECEIVER ESP32
 *
 * Responsibilities:
 *   - Receives CAN frames from Controller ESP32
 *   - Connects to WiFi
 *   - POSTs state to backend every 1 s
 *   - Prints received state to Serial Monitor
 *
 * WIRING — MCP2515 CAN module (VSPI)
 *   MCP2515 VCC  → 3.3V
 *   MCP2515 GND  → GND
 *   MCP2515 SCK  → GPIO 18
 *   MCP2515 SI   → GPIO 23  (MOSI)
 *   MCP2515 SO   → GPIO 19  (MISO)
 *   MCP2515 CS   → GPIO  5
 *   MCP2515 INT  → GPIO  4  (optional interrupt, used here for efficiency)
 *   CAN H / CAN L → matched wiring to Controller module
 *
 * CAN MESSAGES (500 kbps)
 *   ID 0x100 (2 bytes) — boolean flags
 *   ID 0x101 (8 bytes) — bottleCounter(2) + faultCount(2) + uptime(4)
 *
 * LIBRARY: mcp_can by Cory J. Fowler
 *   Arduino IDE → Library Manager → search "mcp_can" → install
 *
 * FreeRTOS Tasks
 *   taskCANReceive (core 1, 10 ms)  — polls MCP2515 for incoming frames
 *   taskComms      (core 0,  1 s)   — WiFi reconnect + HTTP POST
 *   taskSerial     (core 0,  1 s)   — Serial Monitor dump
 */

#include <SPI.h>
#include <mcp_can.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// ─── Config ───────────────────────────────────────────────────────────────────
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* BACKEND_URL   = "http://192.168.1.100:8000/data";  // your PC's IP

// ─── CAN ──────────────────────────────────────────────────────────────────────
#define CAN_CS_PIN   5
#define CAN_INT_PIN  4
#define CAN_SPEED    CAN_500KBPS
#define CAN_CLOCK    MCP_8MHZ        // change to MCP_16MHZ if your module has 16 MHz crystal

MCP_CAN CAN_BUS(CAN_CS_PIN);

#define CAN_ID_FLAGS    0x100
#define CAN_ID_COUNTERS 0x101

// ─── Shared state ─────────────────────────────────────────────────────────────
struct SystemState {
  bool systemRunning    = false;
  bool fault            = false;
  bool emergencyStop    = false;
  bool bottlePresent    = false;
  bool bottlePositioned = false;
  bool fillEnable       = false;
  bool fillingComplete  = false;
  bool runLatch         = false;
  bool conveyorMotor    = false;
  bool valveOn          = false;
  bool greenLED         = false;
  bool yellowLED        = false;
  bool redLED           = false;
  bool buzzerOn         = false;
  int  bottleCounter    = 0;
  int  faultCount       = 0;
  int  uptime           = 0;
};

SystemState       g_state;
SemaphoreHandle_t g_mutex;

// ─── CAN flag byte unpacking ──────────────────────────────────────────────────
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

void unpackCounters(uint8_t* data, SystemState& s) {
  s.bottleCounter = ((uint16_t)data[0] << 8) | data[1];
  s.faultCount    = ((uint16_t)data[2] << 8) | data[3];
  s.uptime        = ((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) |
                    ((uint32_t)data[6] <<  8) |  data[7];
}

// ═════════════════════════════════════════════════════════════════════════════
// TASK: taskCANReceive  (core 1, 10 ms)
// Polls MCP2515 for incoming frames and unpacks into g_state.
// ═════════════════════════════════════════════════════════════════════════════
void taskCANReceive(void* pv) {
  Serial.println("[taskCANReceive] started");

  for (;;) {
    // Check INT pin — LOW means a message is waiting
    if (digitalRead(CAN_INT_PIN) == LOW) {
      long unsigned int rxId;
      unsigned char    rxLen;
      unsigned char    rxBuf[8];

      CAN_BUS.readMsgBuf(&rxId, &rxLen, rxBuf);

      xSemaphoreTake(g_mutex, portMAX_DELAY);

      if (rxId == CAN_ID_FLAGS && rxLen >= 2) {
        unpackFlags(rxBuf[0], rxBuf[1], g_state);
        Serial.printf("[CAN] RX 0x100 — flags: 0x%02X 0x%02X\n", rxBuf[0], rxBuf[1]);
      } else if (rxId == CAN_ID_COUNTERS && rxLen == 8) {
        unpackCounters(rxBuf, g_state);
        Serial.printf("[CAN] RX 0x101 — bottles: %d  faults: %d  uptime: %d\n",
          g_state.bottleCounter, g_state.faultCount, g_state.uptime);
      }

      xSemaphoreGive(g_mutex);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TASK: taskComms  (core 0, 1 s)
// Maintains WiFi and POSTs latest state to the backend.
// ═════════════════════════════════════════════════════════════════════════════
void taskComms(void* pv) {
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.printf("\n[WiFi] Connected — IP: %s\n", WiFi.localIP().toString().c_str());

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Reconnect if dropped
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Reconnecting...");
      WiFi.reconnect();
      vTaskDelay(pdMS_TO_TICKS(2000));
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Still disconnected — retrying next cycle.");
        continue;
      }
      Serial.println("[WiFi] Reconnected.");
    }

    // Snapshot state
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    SystemState s = g_state;
    xSemaphoreGive(g_mutex);

    // Build JSON
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
    http.begin(BACKEND_URL);
    http.addHeader("Content-Type", "application/json");
    int code = http.POST(body);

    if (code > 0) {
      Serial.printf("[HTTP] POST → %d\n", code);
    } else {
      Serial.printf("[HTTP] POST failed: %s\n", http.errorToString(code).c_str());
    }
    http.end();
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TASK: taskSerial  (core 0, 1 s)
// ═════════════════════════════════════════════════════════════════════════════
void taskSerial(void* pv) {
  Serial.println("[taskSerial] started");
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    SystemState s = g_state;
    xSemaphoreGive(g_mutex);

    Serial.println("─────────────────────────────────────");
    Serial.printf("  systemRunning   : %s\n", s.systemRunning    ? "YES" : "NO");
    Serial.printf("  fault           : %s\n", s.fault            ? "YES" : "NO");
    Serial.printf("  emergencyStop   : %s\n", s.emergencyStop    ? "YES" : "NO");
    Serial.printf("  bottlePresent   : %s\n", s.bottlePresent    ? "YES" : "NO");
    Serial.printf("  bottlePositioned: %s\n", s.bottlePositioned ? "YES" : "NO");
    Serial.printf("  fillEnable      : %s\n", s.fillEnable       ? "YES" : "NO");
    Serial.printf("  fillingComplete : %s\n", s.fillingComplete  ? "YES" : "NO");
    Serial.printf("  valveOn         : %s\n", s.valveOn          ? "YES" : "NO");
    Serial.printf("  conveyorMotor   : %s\n", s.conveyorMotor    ? "YES" : "NO");
    Serial.printf("  bottleCounter   : %d\n", s.bottleCounter);
    Serial.printf("  faultCount      : %d\n", s.faultCount);
    Serial.printf("  uptime          : %d s\n", s.uptime);
    Serial.println("─────────────────────────────────────");
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// setup / loop
// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== RECEIVER ESP32 BOOT ===");

  pinMode(CAN_INT_PIN, INPUT);

  // MCP2515 init
  Serial.print("[CAN] Initialising MCP2515...");
  while (CAN_BUS.begin(MCP_ANY, CAN_SPEED, CAN_CLOCK) != CAN_OK) {
    Serial.print(".");
    delay(500);
  }
  CAN_BUS.setMode(MCP_NORMAL);
  Serial.println(" OK");

  g_mutex = xSemaphoreCreateMutex();
  configASSERT(g_mutex);

  xTaskCreatePinnedToCore(taskCANReceive, "CANReceive", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskComms,      "Comms",      8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskSerial,     "Serial",     4096, NULL, 1, NULL, 0);

  Serial.println("=== Ready — waiting for CAN frames ===\n");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
