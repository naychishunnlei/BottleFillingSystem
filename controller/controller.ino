/*
 * Bottle Filling System — CONTROLLER ESP32
 * Scenario A: Automated Bottle Filling System
 *
 * Converts ladder logic rungs to FreeRTOS tasks.
 * Sends system state to Receiver ESP32 via CAN bus (TWAI) every 200 ms.
 *
 * WIRING — CAN Transceiver (SN65HVD230 or TJA1050)
 *   ESP32 GPIO 21 → TX pin of transceiver
 *   ESP32 GPIO 22 → RX pin of transceiver
 *   Transceiver CANH / CANL → matched to Receiver transceiver
 *   Terminate each end of CAN bus with 120Ω resistor across CANH–CANL
 *
 * PIN MAPPING
 *   GPIO 34  Sensor 1 — Bottle presence
 *   GPIO 35  Sensor 2 — Bottle positioned / fill level
 *   GPIO 25  Switch 1 — Start
 *   GPIO 26  Switch 2 — Stop
 *   GPIO 27  Switch 3 — Emergency stop / Reset
 *   GPIO 32  Relay    — Filling valve
 *   GPIO 19  LED Green  — System running
 *   GPIO 18  LED Yellow — Filling in progress
 *   GPIO  5  LED Red    — Error / stopped
 *   GPIO 17  Buzzer     — Cycle complete
 *
 * CAN MESSAGES (500 kbps)
 *   ID 0x100  2 bytes — packed boolean flags
 *   ID 0x101  8 bytes — bottleCounter(u16) + faultCount(u16) + uptime(u32)
 *
 * FreeRTOS Tasks
 *   taskInputs       core 1, 10 ms  — reads sensors & detects switch edges
 *   taskStateMachine core 1, 10 ms  — ladder logic state machine
 *   taskOutputs      core 1, 10 ms  — drives GPIO outputs
 *   taskUptime       core 1,  1 s   — increments uptime while running
 *   taskCAN          core 0, 200 ms — transmits CAN frames
 *   taskSerial       core 0,  1 s   — Serial Monitor state dump
 */

#include "driver/twai.h"

// ─── CAN config ───────────────────────────────────────────────────────────────
#define CAN_TX_PIN  21
#define CAN_RX_PIN  22
#define CAN_ID_FLAGS    0x100
#define CAN_ID_COUNTERS 0x101

// ─── Timing (ladder TON timer values) ────────────────────────────────────────
const TickType_t TON_FILL   = pdMS_TO_TICKS(5000);   // fill duration timer
const TickType_t TON_BUZZER = pdMS_TO_TICKS(500);     // buzzer pulse timer
const TickType_t TON_FAULT  = pdMS_TO_TICKS(15000);   // fault detection timeout

// ─── Pins ─────────────────────────────────────────────────────────────────────
const int PIN_SENSOR1    = 34;  // I0.0 — bottle presence
const int PIN_SENSOR2    = 35;  // I0.1 — bottle positioned
const int PIN_SW_START   = 25;  // I0.2 — start
const int PIN_SW_STOP    = 26;  // I0.3 — stop
const int PIN_SW_EMERG   = 27;  // I0.4 — emergency stop / reset

const int PIN_VALVE      = 32;  // Q0.0 — filling valve relay
const int PIN_LED_GREEN  = 19;  // Q0.1 — system running
const int PIN_LED_YELLOW = 18;  // Q0.2 — filling in progress
const int PIN_LED_RED    = 5;   // Q0.3 — fault / stopped
const int PIN_BUZZER     = 17;  // Q0.4 — cycle complete alert

// ─── Shared state ─────────────────────────────────────────────────────────────
struct SystemState {
  // Physical inputs
  bool bottlePresent    = false;  // I0.0
  bool bottlePositioned = false;  // I0.1
  bool swStart          = false;  // I0.2 edge flag
  bool swStop           = false;  // I0.3 edge flag
  bool swEmergency      = false;  // I0.4 edge flag

  // Internal coils / bits (ladder memory bits)
  bool systemRunning    = false;  // M0.0
  bool fault            = false;  // M0.1
  bool emergencyStop    = false;  // M0.2
  bool fillEnable       = false;  // M0.3
  bool fillingComplete  = false;  // M0.4
  bool runLatch         = false;  // M0.5 — seal-in bit
  bool conveyorMotor    = false;  // M0.6

  // Outputs
  bool valveOn          = false;  // Q0.0
  bool greenLED         = false;  // Q0.1
  bool yellowLED        = false;  // Q0.2
  bool redLED           = true;   // Q0.3 — on at boot
  bool buzzerOn         = false;  // Q0.4

  // Software counter (CTU)
  int  bottleCounter    = 0;
  int  faultCount       = 0;
  int  uptime           = 0;
};

SystemState       g_state;
SemaphoreHandle_t g_mutex;

// ─── CAN flag packing ─────────────────────────────────────────────────────────
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
// Reads physical inputs; detects rising edges on momentary switches.
// ═════════════════════════════════════════════════════════════════════════════
void taskInputs(void* pv) {
  bool lastStart = false, lastStop = false, lastEmerg = false;

  for (;;) {
    bool rawPresent    = digitalRead(PIN_SENSOR1)  == HIGH;
    bool rawPositioned = digitalRead(PIN_SENSOR2)  == HIGH;
    bool rawStart      = digitalRead(PIN_SW_START) == HIGH;
    bool rawStop       = digitalRead(PIN_SW_STOP)  == HIGH;
    bool rawEmerg      = digitalRead(PIN_SW_EMERG) == HIGH;

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_state.bottlePresent    = rawPresent;
    g_state.bottlePositioned = rawPositioned;
    if (rawStart && !lastStart) g_state.swStart     = true;
    if (rawStop  && !lastStop)  g_state.swStop      = true;
    if (rawEmerg && !lastEmerg) g_state.swEmergency = true;
    xSemaphoreGive(g_mutex);

    lastStart = rawStart;
    lastStop  = rawStop;
    lastEmerg = rawEmerg;

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TASK: taskStateMachine  (core 1, 10 ms)
// Ladder-to-code conversion — each block below maps to a ladder rung.
// ═════════════════════════════════════════════════════════════════════════════
void taskStateMachine(void* pv) {
  TickType_t fillStart          = 0;
  TickType_t buzzerStart        = 0;
  TickType_t bottlePresentStart = 0;
  bool       buzzerPending      = false;

  for (;;) {
    TickType_t now = xTaskGetTickCount();

    xSemaphoreTake(g_mutex, portMAX_DELAY);

    bool doStart  = g_state.swStart;     g_state.swStart     = false;
    bool doStop   = g_state.swStop;      g_state.swStop      = false;
    bool doEmerg  = g_state.swEmergency; g_state.swEmergency = false;
    bool present    = g_state.bottlePresent;
    bool positioned = g_state.bottlePositioned;

    // ── Rung 1: Emergency stop override (highest priority) ───────────────────
    // Ladder: [I0.4] ──┬── (M0.2) Emergency stop SET
    //                  └── [/M0.2] ── (M0.2 RESET, M0.1 RESET)
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
        Serial.println("[!!!] EMERGENCY STOP");
      } else {
        // Second press = reset
        g_state.emergencyStop   = false;
        g_state.fault           = false;
        g_state.fillingComplete = false;
        g_state.fillEnable      = false;
        bottlePresentStart      = 0;
        Serial.println("[RST] Reset — press START to run");
      }
    }

    // ── Rung 2: Stop logic ────────────────────────────────────────────────────
    // Ladder: [I0.3][M0.0] ── (M0.0 RESET, M0.5 RESET)
    if (doStop && g_state.systemRunning) {
      g_state.systemRunning = false;
      g_state.runLatch      = false;
      g_state.valveOn       = false;
      g_state.conveyorMotor = false;
      g_state.buzzerOn      = false;
      buzzerPending         = false;
      Serial.println("[STP] Stopped");
    }

    // ── Rung 3: Start / seal-in latch ────────────────────────────────────────
    // Ladder: [I0.2 + M0.5][/M0.2][/M0.1] ── (M0.0, M0.5)
    if (doStart && !g_state.systemRunning && !g_state.emergencyStop) {
      g_state.systemRunning = true;
      g_state.runLatch      = true;
      g_state.fault         = false;
      bottlePresentStart    = 0;
      Serial.println("[RUN] Running");
    }

    // ── Rungs 4–7: Filling sequence (executes only when M0.0 = 1) ────────────
    if (g_state.systemRunning && !g_state.fault && !g_state.emergencyStop) {
      g_state.conveyorMotor = true;

      // Rung 4: Open valve when bottle detected in position
      // Ladder: [M0.0][I0.0][I0.1][/M0.3][/M0.4] ── (Q0.0, M0.3) TON start
      if (present && positioned && !g_state.fillEnable && !g_state.fillingComplete) {
        g_state.fillEnable = true;
        g_state.valveOn    = true;
        fillStart          = now;
        bottlePresentStart = 0;
        Serial.printf("[FIL] Filling #%d\n", g_state.bottleCounter + 1);
      }

      // Rung 5: TON done — close valve, increment CTU counter
      // Ladder: [M0.3][TON >= 5s] ── (Q0.0 RESET, M0.4, CTU++)
      if (g_state.fillEnable && g_state.valveOn && (now - fillStart) >= TON_FILL) {
        g_state.valveOn         = false;
        g_state.fillingComplete = true;
        g_state.fillEnable      = false;
        g_state.bottleCounter++;          // CTU count up
        g_state.buzzerOn        = true;
        buzzerPending           = true;
        buzzerStart             = now;
        Serial.printf("[DON] Bottle #%d done\n", g_state.bottleCounter);
      }

      // Rung 6: Bottle removed — reset fill complete, ready for next cycle
      // Ladder: [M0.4][/I0.0] ── (M0.4 RESET)
      if (g_state.fillingComplete && !present) {
        g_state.fillingComplete = false;
        bottlePresentStart      = 0;
        Serial.println("[NXT] Ready");
      }

      // Rung 7: Fault — bottle present but not positioned within timeout
      // Ladder: [I0.0][/I0.1][/M0.3][TON >= 15s] ── (M0.1, CTU_fault++)
      if (present && !positioned && !g_state.fillEnable && !g_state.fillingComplete) {
        if (bottlePresentStart == 0) bottlePresentStart = now;
        if ((now - bottlePresentStart) >= TON_FAULT) {
          g_state.fault      = true;
          g_state.valveOn    = false;
          g_state.faultCount++;
          bottlePresentStart = 0;
          Serial.println("[ERR] Fault: bottle not positioned");
        }
      } else {
        if (!g_state.fillEnable && !present) bottlePresentStart = 0;
      }

    } else {
      g_state.conveyorMotor = false;
      if (!g_state.fillingComplete) g_state.valveOn = false;
    }

    // ── Rung 8: Buzzer TOF timer ──────────────────────────────────────────────
    // Ladder: [Q0.4][TON_buzzer >= 500ms] ── (Q0.4 RESET)
    if (buzzerPending && g_state.buzzerOn && (now - buzzerStart) >= TON_BUZZER) {
      g_state.buzzerOn = false;
      buzzerPending    = false;
    }

    xSemaphoreGive(g_mutex);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TASK: taskOutputs  (core 1, 10 ms)
// Derives LED states from logic bits and drives all GPIO outputs.
// ═════════════════════════════════════════════════════════════════════════════
void taskOutputs(void* pv) {
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

    digitalWrite(PIN_VALVE,      valve  ? HIGH : LOW);
    digitalWrite(PIN_LED_GREEN,  green  ? HIGH : LOW);
    digitalWrite(PIN_LED_YELLOW, yellow ? HIGH : LOW);
    digitalWrite(PIN_LED_RED,    red    ? HIGH : LOW);
    digitalWrite(PIN_BUZZER,     buzz   ? HIGH : LOW);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TASK: taskUptime  (core 1, 1 s)
// Software counter — increments only while system is running.
// ═════════════════════════════════════════════════════════════════════════════
void taskUptime(void* pv) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    if (g_state.systemRunning) g_state.uptime++;
    xSemaphoreGive(g_mutex);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TASK: taskCAN  (core 0, 200 ms)
// Transmits two CAN frames to Receiver ESP32.
//   0x100 — 2-byte packed boolean flags
//   0x101 — 8-byte counters
// ═════════════════════════════════════════════════════════════════════════════
void taskCAN(void* pv) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(200));

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    SystemState s = g_state;
    xSemaphoreGive(g_mutex);

    // Frame 0x100 — flags
    twai_message_t msg0 = {};
    msg0.identifier       = CAN_ID_FLAGS;
    msg0.data_length_code = 2;
    msg0.data[0]          = packFlags0(s);
    msg0.data[1]          = packFlags1(s);
    esp_err_t r0 = twai_transmit(&msg0, pdMS_TO_TICKS(10));
    if (r0 != ESP_OK) Serial.printf("[CAN] TX 0x100 err: %d\n", r0);

    // Frame 0x101 — counters (big-endian)
    twai_message_t msg1 = {};
    msg1.identifier       = CAN_ID_COUNTERS;
    msg1.data_length_code = 8;
    msg1.data[0] = (s.bottleCounter >> 8) & 0xFF;
    msg1.data[1] =  s.bottleCounter       & 0xFF;
    msg1.data[2] = (s.faultCount    >> 8) & 0xFF;
    msg1.data[3] =  s.faultCount          & 0xFF;
    msg1.data[4] = (s.uptime >> 24) & 0xFF;
    msg1.data[5] = (s.uptime >> 16) & 0xFF;
    msg1.data[6] = (s.uptime >>  8) & 0xFF;
    msg1.data[7] =  s.uptime        & 0xFF;
    esp_err_t r1 = twai_transmit(&msg1, pdMS_TO_TICKS(10));
    if (r1 != ESP_OK) Serial.printf("[CAN] TX 0x101 err: %d\n", r1);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// TASK: taskSerial  (core 0, 1 s)
// ═════════════════════════════════════════════════════════════════════════════
void taskSerial(void* pv) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    SystemState s = g_state;
    xSemaphoreGive(g_mutex);

    Serial.println("---");
    Serial.printf("running:%d fault:%d emergency:%d\n",  s.systemRunning, s.fault, s.emergencyStop);
    Serial.printf("present:%d positioned:%d valve:%d\n", s.bottlePresent, s.bottlePositioned, s.valveOn);
    Serial.printf("fill:%d complete:%d motor:%d\n",      s.fillEnable, s.fillingComplete, s.conveyorMotor);
    Serial.printf("bottles:%d faults:%d uptime:%ds\n",   s.bottleCounter, s.faultCount, s.uptime);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// setup / loop
// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== CONTROLLER ESP32 BOOT ===");

  pinMode(PIN_SENSOR1,    INPUT);
  pinMode(PIN_SENSOR2,    INPUT);
  pinMode(PIN_SW_START,   INPUT_PULLDOWN);
  pinMode(PIN_SW_STOP,    INPUT_PULLDOWN);
  pinMode(PIN_SW_EMERG,   INPUT_PULLDOWN);
  pinMode(PIN_VALVE,      OUTPUT);
  pinMode(PIN_LED_GREEN,  OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED,    OUTPUT);
  pinMode(PIN_BUZZER,     OUTPUT);

  digitalWrite(PIN_VALVE,      LOW);
  digitalWrite(PIN_LED_GREEN,  LOW);
  digitalWrite(PIN_LED_YELLOW, LOW);
  digitalWrite(PIN_LED_RED,    HIGH);
  digitalWrite(PIN_BUZZER,     LOW);

  // TWAI (CAN) init
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

  xTaskCreatePinnedToCore(taskInputs,       "Inputs",       2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskStateMachine, "StateMachine", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskOutputs,      "Outputs",      2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskUptime,       "Uptime",       1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskCAN,          "CAN",          4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskSerial,       "Serial",       4096, NULL, 1, NULL, 0);

  Serial.println("=== All tasks started. Press START to begin. ===\n");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
