/*
 * Bottle Filling System — CONTROLLER ESP32
 *
 * Responsibilities:
 *   - Reads all sensors and switches
 *   - Runs filling state machine
 *   - Drives all actuators (valve, LEDs, buzzer)
 *   - Sends state to Receiver ESP32 via CAN bus every 200 ms
 *   - No WiFi needed
 *
 * WIRING — MCP2515 CAN module (VSPI)
 *   MCP2515 VCC  → 3.3V
 *   MCP2515 GND  → GND
 *   MCP2515 SCK  → GPIO 18
 *   MCP2515 SI   → GPIO 23  (MOSI)
 *   MCP2515 SO   → GPIO 19  (MISO)
 *   MCP2515 CS   → GPIO  5
 *   MCP2515 INT  → GPIO  4  (optional, not used here)
 *   CAN H / CAN L → matched wiring to Receiver module
 *
 * NOTE: GPIO 18, 19 are used by SPI (MCP2515).
 *       LED pins are shifted to avoid conflict — see below.
 *
 * PIN MAPPING
 *  Inputs
 *    GPIO 34  Sensor 1 — Bottle presence
 *    GPIO 35  Sensor 2 — Bottle positioned
 *    GPIO 25  Switch 1 — Start
 *    GPIO 26  Switch 2 — Stop
 *    GPIO 27  Switch 3 — Emergency stop / Reset
 *
 *  Outputs
 *    GPIO 32  Relay    — Filling valve
 *    GPIO 33  LED Green  — System running
 *    GPIO 16  LED Yellow — Filling in progress
 *    GPIO 17  LED Red    — Error / stopped
 *    GPIO 15  Buzzer     — Cycle complete
 *
 * CAN MESSAGES (500 kbps)
 *   ID 0x100 (2 bytes) — boolean flags
 *   ID 0x101 (8 bytes) — bottleCounter(2) + faultCount(2) + uptime(4)
 *
 * LIBRARY: mcp_can by Cory J. Fowler
 *   Arduino IDE → Library Manager → search "mcp_can" → install
 *
 * FreeRTOS Tasks
 *   taskInputs       (core 1, 10 ms)  — sensors & switch edges
 *   taskStateMachine (core 1, 10 ms)  — filling logic
 *   taskOutputs      (core 1, 10 ms)  — drives GPIO
 *   taskUptime       (core 1,  1 s)   — uptime counter
 *   taskCAN          (core 0, 200 ms) — sends CAN frames
 *   taskSerial       (core 0,  1 s)   — Serial Monitor dump
 */

#include <SPI.h>
#include <mcp_can.h>

// ─── CAN ──────────────────────────────────────────────────────────────────────
#define CAN_CS_PIN   5
#define CAN_SPEED    CAN_500KBPS
#define CAN_CLOCK    MCP_8MHZ        // change to MCP_16MHZ if your module has 16 MHz crystal

MCP_CAN CAN_BUS(CAN_CS_PIN);

// CAN message IDs
#define CAN_ID_FLAGS    0x100
#define CAN_ID_COUNTERS 0x101

// ─── Timing ───────────────────────────────────────────────────────────────────
const TickType_t FILL_TICKS   = pdMS_TO_TICKS(5000);
const TickType_t BUZZER_TICKS = pdMS_TO_TICKS(500);
const TickType_t FAULT_TICKS  = pdMS_TO_TICKS(15000);
const TickType_t CAN_TICKS    = pdMS_TO_TICKS(200);

// ─── Pins ─────────────────────────────────────────────────────────────────────
const int PIN_SENSOR_PRESENCE = 34;
const int PIN_SENSOR_LEVEL    = 35;
const int PIN_SW_START        = 25;
const int PIN_SW_STOP         = 26;
const int PIN_SW_EMERGENCY    = 27;

const int PIN_RELAY_VALVE     = 32;
const int PIN_LED_GREEN       = 33;  // shifted — 19 used by VSPI MISO
const int PIN_LED_YELLOW      = 16;  // shifted — 18 used by VSPI SCK
const int PIN_LED_RED         = 17;
const int PIN_BUZZER          = 15;

// ─── Shared state ─────────────────────────────────────────────────────────────
struct SystemState {
  // Sensors / switches (written by taskInputs)
  bool bottlePresent    = false;
  bool bottlePositioned = false;
  bool swStart          = false;
  bool swStop           = false;
  bool swEmergency      = false;

  // Logic (written by taskStateMachine)
  bool systemRunning    = false;
  bool fault            = false;
  bool emergencyStop    = false;
  bool fillEnable       = false;
  bool fillingComplete  = false;
  bool runLatch         = false;
  bool conveyorMotor    = false;
  bool valveOn          = false;

  // Outputs (written by taskOutputs)
  bool greenLED         = false;
  bool yellowLED        = false;
  bool redLED           = true;
  bool buzzerOn         = false;

  // Counters
  int  bottleCounter    = 0;
  int  faultCount       = 0;
  int  uptime           = 0;
};

SystemState       g_state;
SemaphoreHandle_t g_mutex;

// ─── CAN flag byte packing ────────────────────────────────────────────────────
//  Byte 0: [systemRunning | fault | emergencyStop | bottlePresent |
//            bottlePositioned | fillEnable | fillingComplete | runLatch]
//  Byte 1: [conveyorMotor | valveOn | greenLED | yellowLED |
//            redLED | buzzerOn | 0 | 0]
uint8_t packFlags0(const SystemState& s) {
  return (s.systemRunning    << 7) | (s.fault            << 6) |
         (s.emergencyStop    << 5) | (s.bottlePresent    << 4) |
         (s.bottlePositioned << 3) | (s.fillEnable       << 2) |
         (s.fillingComplete  << 1) | (s.runLatch         << 0);
}
uint8_t packFlags1(const SystemState& s) {
  return (s.conveyorMotor << 7) | (s.valveOn    << 6) |
         (s.greenLED      << 5) | (s.yellowLED  << 4) |
         (s.redLED        << 3) | (s.buzzerOn   << 2);
}

// ═════════════════════════════════════════════════════════════════════════════
// TASK: taskInputs  (core 1, 10 ms)
// ═════════════════════════════════════════════════════════════════════════════
void taskInputs(void* pv) {
  bool lastStart = false, lastStop = false, lastEmerg = false;
  Serial.println("[taskInputs] started");

  for (;;) {
    bool rawPresent    = digitalRead(PIN_SENSOR_PRESENCE) == HIGH;
    bool rawPositioned = digitalRead(PIN_SENSOR_LEVEL)    == HIGH;
    bool rawStart      = digitalRead(PIN_SW_START)        == HIGH;
    bool rawStop       = digitalRead(PIN_SW_STOP)         == HIGH;
    bool rawEmerg      = digitalRead(PIN_SW_EMERGENCY)    == HIGH;

    bool edgeStart = rawStart && !lastStart;
    bool edgeStop  = rawStop  && !lastStop;
    bool edgeEmerg = rawEmerg && !lastEmerg;

    lastStart = rawStart;
    lastStop  = rawStop;
    lastEmerg = rawEmerg;

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_state.bottlePresent    = rawPresent;
    g_state.bottlePositioned = rawPositioned;
    if (edgeStart) g_state.swStart     = true;
    if (edgeStop)  g_state.swStop      = true;
    if (edgeEmerg) g_state.swEmergency = true;
    xSemaphoreGive(g_mutex);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TASK: taskStateMachine  (core 1, 10 ms)
// ═════════════════════════════════════════════════════════════════════════════
void taskStateMachine(void* pv) {
  TickType_t fillStart          = 0;
  TickType_t buzzerStart        = 0;
  TickType_t bottlePresentStart = 0;
  bool       buzzerPending      = false;
  Serial.println("[taskStateMachine] started");

  for (;;) {
    TickType_t now = xTaskGetTickCount();

    xSemaphoreTake(g_mutex, portMAX_DELAY);

    bool doStart  = g_state.swStart;     g_state.swStart     = false;
    bool doStop   = g_state.swStop;      g_state.swStop      = false;
    bool doEmerg  = g_state.swEmergency; g_state.swEmergency = false;
    bool present    = g_state.bottlePresent;
    bool positioned = g_state.bottlePositioned;

    // Emergency stop / Reset
    if (doEmerg) {
      if (!g_state.emergencyStop) {
        g_state.emergencyStop = true;
        g_state.systemRunning = false;
        g_state.runLatch      = false;
        g_state.fault         = true;
        g_state.valveOn       = false;
        g_state.conveyorMotor = false;
        g_state.buzzerOn      = false;
        g_state.faultCount++;
        buzzerPending = false;
        Serial.println("[!!!] EMERGENCY STOP triggered!");
      } else {
        g_state.emergencyStop   = false;
        g_state.fault           = false;
        g_state.fillingComplete = false;
        g_state.fillEnable      = false;
        bottlePresentStart      = 0;
        Serial.println("[RST] System reset. Press START to run.");
      }
    }

    // Stop
    if (doStop && g_state.systemRunning) {
      g_state.systemRunning = false;
      g_state.runLatch      = false;
      g_state.valveOn       = false;
      g_state.conveyorMotor = false;
      g_state.buzzerOn      = false;
      buzzerPending         = false;
      Serial.println("[STP] Stop pressed — system halted.");
    }

    // Start
    if (doStart && !g_state.systemRunning && !g_state.emergencyStop) {
      g_state.systemRunning = true;
      g_state.runLatch      = true;
      g_state.fault         = false;
      bottlePresentStart    = 0;
      Serial.println("[RUN] Start pressed — system running.");
    }

    // Filling logic
    if (g_state.systemRunning && !g_state.fault && !g_state.emergencyStop) {
      g_state.conveyorMotor = true;

      if (present && positioned && !g_state.fillEnable && !g_state.fillingComplete) {
        g_state.fillEnable = true;
        g_state.valveOn    = true;
        fillStart          = now;
        bottlePresentStart = 0;
        Serial.printf("[FIL] Filling bottle #%d...\n", g_state.bottleCounter + 1);
      }

      if (g_state.fillEnable && g_state.valveOn && (now - fillStart) >= FILL_TICKS) {
        g_state.valveOn         = false;
        g_state.fillingComplete = true;
        g_state.fillEnable      = false;
        g_state.bottleCounter++;
        g_state.buzzerOn        = true;
        buzzerPending           = true;
        buzzerStart             = now;
        Serial.printf("[DON] Fill complete — total: %d bottles\n", g_state.bottleCounter);
      }

      if (g_state.fillingComplete && !present) {
        g_state.fillingComplete = false;
        bottlePresentStart      = 0;
        Serial.println("[NXT] Bottle removed — ready for next.");
      }

      if (present && !positioned && !g_state.fillEnable && !g_state.fillingComplete) {
        if (bottlePresentStart == 0) bottlePresentStart = now;
        if ((now - bottlePresentStart) >= FAULT_TICKS) {
          g_state.fault      = true;
          g_state.valveOn    = false;
          g_state.faultCount++;
          bottlePresentStart = 0;
          Serial.println("[ERR] Fault: bottle not positioned in time.");
        }
      } else {
        if (!g_state.fillEnable && !present) bottlePresentStart = 0;
      }

    } else {
      g_state.conveyorMotor = false;
      if (!g_state.fillingComplete) g_state.valveOn = false;
    }

    // Buzzer auto-off
    if (buzzerPending && g_state.buzzerOn && (now - buzzerStart) >= BUZZER_TICKS) {
      g_state.buzzerOn = false;
      buzzerPending    = false;
    }

    xSemaphoreGive(g_mutex);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TASK: taskOutputs  (core 1, 10 ms)
// ═════════════════════════════════════════════════════════════════════════════
void taskOutputs(void* pv) {
  Serial.println("[taskOutputs] started");

  for (;;) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_state.greenLED  =  g_state.systemRunning && !g_state.fault && !g_state.emergencyStop;
    g_state.yellowLED =  g_state.valveOn;
    g_state.redLED    = !g_state.systemRunning || g_state.fault || g_state.emergencyStop;

    bool valve  = g_state.valveOn;
    bool green  = g_state.greenLED;
    bool yellow = g_state.yellowLED;
    bool red    = g_state.redLED;
    bool buzz   = g_state.buzzerOn;
    xSemaphoreGive(g_mutex);

    digitalWrite(PIN_RELAY_VALVE,  valve  ? HIGH : LOW);
    digitalWrite(PIN_LED_GREEN,    green  ? HIGH : LOW);
    digitalWrite(PIN_LED_YELLOW,   yellow ? HIGH : LOW);
    digitalWrite(PIN_LED_RED,      red    ? HIGH : LOW);
    digitalWrite(PIN_BUZZER,       buzz   ? HIGH : LOW);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TASK: taskUptime  (core 1, 1 s)
// ═════════════════════════════════════════════════════════════════════════════
void taskUptime(void* pv) {
  Serial.println("[taskUptime] started");
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    if (g_state.systemRunning) g_state.uptime++;
    xSemaphoreGive(g_mutex);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TASK: taskCAN  (core 0, 200 ms)
// Sends two CAN frames:
//   0x100 — 2-byte packed boolean flags
//   0x101 — 8-byte counters (bottleCounter, faultCount, uptime)
// ═════════════════════════════════════════════════════════════════════════════
void taskCAN(void* pv) {
  Serial.println("[taskCAN] started");

  for (;;) {
    vTaskDelay(CAN_TICKS);

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    SystemState s = g_state;
    xSemaphoreGive(g_mutex);

    // Frame 0x100 — flags (2 bytes)
    uint8_t flags[2] = { packFlags0(s), packFlags1(s) };
    byte result = CAN_BUS.sendMsgBuf(CAN_ID_FLAGS, 0, 2, flags);
    if (result != CAN_OK) {
      Serial.printf("[CAN] Send 0x100 failed: %d\n", result);
    }

    // Frame 0x101 — counters (8 bytes, big-endian)
    uint8_t counters[8];
    counters[0] = (s.bottleCounter >> 8) & 0xFF;
    counters[1] =  s.bottleCounter       & 0xFF;
    counters[2] = (s.faultCount    >> 8) & 0xFF;
    counters[3] =  s.faultCount          & 0xFF;
    counters[4] = (s.uptime >> 24) & 0xFF;
    counters[5] = (s.uptime >> 16) & 0xFF;
    counters[6] = (s.uptime >>  8) & 0xFF;
    counters[7] =  s.uptime        & 0xFF;
    result = CAN_BUS.sendMsgBuf(CAN_ID_COUNTERS, 0, 8, counters);
    if (result != CAN_OK) {
      Serial.printf("[CAN] Send 0x101 failed: %d\n", result);
    }

    Serial.printf("[CAN] Sent — flags: 0x%02X 0x%02X  bottles: %d  faults: %d  uptime: %d\n",
      flags[0], flags[1], s.bottleCounter, s.faultCount, s.uptime);
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
  Serial.println("\n=== CONTROLLER ESP32 BOOT ===");

  // GPIO
  pinMode(PIN_SENSOR_PRESENCE, INPUT);
  pinMode(PIN_SENSOR_LEVEL,    INPUT);
  pinMode(PIN_SW_START,        INPUT_PULLDOWN);
  pinMode(PIN_SW_STOP,         INPUT_PULLDOWN);
  pinMode(PIN_SW_EMERGENCY,    INPUT_PULLDOWN);
  pinMode(PIN_RELAY_VALVE,     OUTPUT);
  pinMode(PIN_LED_GREEN,       OUTPUT);
  pinMode(PIN_LED_YELLOW,      OUTPUT);
  pinMode(PIN_LED_RED,         OUTPUT);
  pinMode(PIN_BUZZER,          OUTPUT);

  digitalWrite(PIN_RELAY_VALVE,  LOW);
  digitalWrite(PIN_LED_GREEN,    LOW);
  digitalWrite(PIN_LED_YELLOW,   LOW);
  digitalWrite(PIN_LED_RED,      HIGH);  // red on at boot
  digitalWrite(PIN_BUZZER,       LOW);

  // MCP2515 init — retry until ready
  Serial.print("[CAN] Initialising MCP2515...");
  while (CAN_BUS.begin(MCP_ANY, CAN_SPEED, CAN_CLOCK) != CAN_OK) {
    Serial.print(".");
    delay(500);
  }
  CAN_BUS.setMode(MCP_NORMAL);
  Serial.println(" OK");

  // Mutex
  g_mutex = xSemaphoreCreateMutex();
  configASSERT(g_mutex);

  // Tasks
  xTaskCreatePinnedToCore(taskInputs,       "Inputs",       2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskStateMachine, "StateMachine", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskOutputs,      "Outputs",      2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskUptime,       "Uptime",       1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskCAN,          "CAN",          4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskSerial,       "Serial",       4096, NULL, 1, NULL, 0);

  Serial.println("=== All tasks created. Press START to begin. ===\n");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
