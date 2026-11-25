#include <Arduino.h>
#include <Wire.h>
#include <esp_task_wdt.h>

#include "config.h"
#include "storage.h"
#include "mpu_sensor.h"
#include "inputs.h"
#include "relays.h"
#include "supervisor.h"
#include "web_portal.h"

void setup() {
  Serial.begin(115200);

  // FAIL-SAFE BOOT GUARD: Immediately force all relay pins LOW on startup/WDT reset
  for (int i = 0; i < NUM_RELAYS; i++) {
    pinMode(relays[i]->value, OUTPUT);
    digitalWrite(relays[i]->value, LOW);
    relays[i]->isOn = false;
  }

  // FAIL-SAFE SHUTDOWN GUARD:  Immediately force all relay pins LOW on clean shutdown
  esp_register_shutdown_handler(groundAllRelaysFailSafe);

  // Initialize Hardware Task Watchdog (3-second timeout, panic/reboot enabled)
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms     = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  esp_task_wdt_init(&wdtConfig);
  Serial.printf("[WDT] Fail-Safe Hardware Watchdog initialized (%u second timeout).\n", WDT_TIMEOUT_SEC);

  Serial.println("[SETUP] Launching multi-core safety vehicle controller initialization sequences...");

  // Load User Configuration Settings from NVS Flash
  loadSystemConfig();

  // Initialize physical I2C system bus tracks (SDA = GPIO 21, SCL = GPIO 22)
  Wire.begin(21, 22);

  // Core Sensor Handshake: Query MPU6050 and load calibration parameters
  initializeMPUCalibration();

  // --- 1. THREAD-SAFE CONCURRENCY LAYER CREATION ---
  brakeQueue         = xQueueCreate(1, sizeof(SignalCommand));
  turnQueue          = xQueueCreate(1, sizeof(SignalCommand));
  hazardQueue        = xQueueCreate(1, sizeof(SignalCommand));
  ignitionQueue      = xQueueCreate(1, sizeof(SignalCommand));
  hornQueue          = xQueueCreate(1, sizeof(bool));
  turnCancelQueue    = xQueueCreate(1, sizeof(SignalCommand));
  calibrationQueue   = xQueueCreate(1, sizeof(bool));
  turnToBrakeQueue   = xQueueCreate(1, sizeof(SignalCommand));
  brakeToTurnQueue   = xQueueCreate(1, sizeof(SignalCommand));
  brakeToHazardQueue = xQueueCreate(1, sizeof(SignalCommand));
  hazardToBrakeQueue = xQueueCreate(1, sizeof(SignalCommand));
  turnToHazardQueue  = xQueueCreate(1, sizeof(SignalCommand));
  hazardToTurnQueue  = xQueueCreate(1, sizeof(SignalCommand));

  SignalCommand initialTurn   = CMD_TURN_OFF;
  SignalCommand initialBrake  = CMD_BRAKE_OFF;
  SignalCommand initialHazard = CMD_HAZARD_OFF;
  bool initialHornState       = false;
  bool initialCalibState      = false;

  xQueueOverwrite(turnQueue, &initialTurn);
  xQueueOverwrite(brakeQueue, &initialBrake);
  xQueueOverwrite(hazardQueue, &initialHazard);
  xQueueOverwrite(hornQueue, &initialHornState);
  xQueueOverwrite(turnCancelQueue, &initialTurn);
  xQueueOverwrite(calibrationQueue, &initialCalibState);
  xQueueOverwrite(turnToBrakeQueue, &initialTurn);
  xQueueOverwrite(brakeToTurnQueue, &initialBrake);
  xQueueOverwrite(brakeToHazardQueue, &initialBrake);
  xQueueOverwrite(hazardToBrakeQueue, &initialHazard);
  xQueueOverwrite(turnToHazardQueue, &initialTurn);
  xQueueOverwrite(hazardToTurnQueue, &initialHazard);


  // --- 2. HARDWARE IO PROVISIONING LAYER ---
  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(buttons[i]->value, INPUT_PULLUP);
    attachInterruptArg(digitalPinToInterrupt(buttons[i]->value), buttonISR, (void*)buttons[i], CHANGE);
  }

  for (int i = 0; i < NUM_RELAYS; i++) {
    pinMode(relays[i]->value, OUTPUT);
    digitalWrite(relays[i]->value, LOW);
  }

  SignalCommand initialIgnition = (digitalRead(Ignition.value) == LOW) ? CMD_IGNITION_ON : CMD_IGNITION_OFF;
  xQueueOverwrite(ignitionQueue, &initialIgnition);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  // Start web server
  startWebPortal();

  // --- 3. BACKGROUND MULTI-TASKING SCHEDULER LAUNCH ---
  xTaskCreatePinnedToCore(processVehicleTelemetry, "processVehicleTelemetry", 3072, NULL, 1, &telemetryHandle, 1);


  xTaskCreatePinnedToCore(mainInputProcessorTask, "mainInputTask", 2048, NULL, 2, &inputTaskHandle, 1);
  xTaskCreatePinnedToCore(turnSignalTask, "turnSignalTask", 1536, NULL, 1, &turnTaskHandle, 0);
  xTaskCreatePinnedToCore(hazardTask, "hazardTask", 1536, NULL, 1, &hazardTaskHandle, 0);
  xTaskCreatePinnedToCore(hornTask, "hornTask", 1536, NULL, 1, &hornTaskHandle, 0);
  Serial.println("[SETUP] Horn relay task successfully pinned and launched on Core 0.");

  if (useBrakelights) {
    xTaskCreatePinnedToCore(brakeLightTask, "brakeLightTask", 1536, NULL, 1, &brakeTaskHandle, 0);
    Serial.println("[SETUP] Brake light relay management task successfully pinned and launched on Core 0.");
  } else {
    Serial.println("[SETUP] Brake light relay management task disabled globally. Skipping runtime creation.");
  }

  if (diagnosticsOn) {
    xTaskCreatePinnedToCore(systemDiagnosticsTask, "diagnosticsTask", 2048, NULL, 0, &diagnosticsTaskHandle, 1);
    Serial.println("[SETUP] System diagnostics task successfully pinned and launched on Core 1.");
  } else {
    Serial.println("[SETUP] System diagnostics task disabled globally. Skipping runtime creation.");
  }

  xTaskCreatePinnedToCore(core0WatchdogTask, "core0WdtTask", 2048, NULL, 1, &core0WdtHandle, 0);
  Serial.println("[SETUP] Core initialization sequences completed successfully. Relinquishing Arduino loop framework.");

  if (useIgnitionInterlock) {
    xTaskCreatePinnedToCore(processSystemInterlockSupervisor, "supervisorTask", 3072, NULL, 3, NULL, 1);
    Serial.println("[SETUP] Ignition Interlock Supervisor task pinned and launched on Core 1.");
  } else {

    // if not using IgnitionInterlock subscribe tasks to hardware watchdog
    if (telemetryHandle != NULL) esp_task_wdt_add(telemetryHandle);
    if (inputTaskHandle != NULL) esp_task_wdt_add(inputTaskHandle);
    if (core0WdtHandle  != NULL) esp_task_wdt_add(core0WdtHandle);

    Serial.println("[SETUP] Ignition Interlock disabled. System will remain permanently active.");
  }
}
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}