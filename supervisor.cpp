#include "supervisor.h"
#include "mpu_sensor.h"
#include "web_portal.h"
#include <esp_sleep.h>
#include <esp_task_wdt.h>

/**
  * @brief Central Supervisor Loop running on Core 1 matching state-aware philosophy.
  * Coordinates multi-core thread lifecycles and physical hardware power gates across three
  * discrete state profiles based on real-time Ignition and Hazard mailbox cues.
  * 
  * Execution States:
  *  - STATE 1 [Full Operational Profile]: Ignition is active. All system tasks, dynamic 
  *            telemetry modules, and output flasher channels are fully operational.
  *  - STATE 2 [Stationary Hazard Profile]: Ignition is inactive, but Hazard lights are enabled.
  *            Brake and Turn tracking engines are frozen to preserve vehicle battery while preserving 
  *            the background hazard task execution engine on Core 0.
  *  - STATE 3 [Ultra-Low Power Deep Standby]: Both Ignition and Hazards are inactive. 
  *            All application threads are suspended, high-current outputs are grounded, and the MPU6050 
  *            register engine is put to sleep before the system triggers a light sleep block.
  * 
  * @param pvParameters Standard untyped FreeRTOS task argument block pointer (Unused).
*/
void processSystemInterlockSupervisor(void* pvParameters) {
  // Edge-detection tracking baselines to prevent redundant state thrashing
  static uint8_t activeStateProfile = 0; // Tracks running profile (1, 2, or 3)
  
  // Local state cache variables initialized to safe boot baselines
  ignitionMode = CMD_IGNITION_OFF;
  SignalCommand currentHazardMode = CMD_HAZARD_OFF;

  for (;;) {

    // 1. DATA PIPELINE ACQUISITION LAYER
    // Non-blocking fetch (0-timeout) to drain the latest status tokens from Core 1 input tasks
    xQueueReceive(ignitionQueue, &ignitionMode, 0);
    xQueueReceive(hazardQueue, &currentHazardMode, 0);

    bool isIgnitionActive = (ignitionMode == CMD_IGNITION_ON);
    bool isHazardActive   = (currentHazardMode == CMD_HAZARD);

    // Evaluate target operation profile based on prioritized conditional mapping
    uint8_t targetStateProfile = 3; // Default fallback to deep standby

    if (isIgnitionActive) {
      targetStateProfile = 1; // Ignition ON takes absolute priority regardless of hazards
    } else if (isHazardActive) {
      targetStateProfile = 2; // Hazards ON without Ignition ON
    }

    // 2. EDGE-GATED STATE MIGRATION EXECUTION MATRIX
    if (targetStateProfile != activeStateProfile) {
      Serial.printf("[SUPERVISOR] State Transition Detected! Migrating Profile: State %d -> State %d\n", 
                    activeStateProfile, targetStateProfile);
      
      // ==========================================================================
      // HIDDEN CALIBRATION INTERCEPT
      // ==========================================================================
      // If moving into STATE 1, check if BOTH blinker inputs are being actively held down.
      // Utilizing digitalRead directly here ensures an instant hardware check on wakeup.
      if (targetStateProfile == 1) {
        bool leftHeld  = (digitalRead(Left.value) == LOW);  // Active-LOW check
        bool rightHeld = (digitalRead(Right.value) == LOW); // Active-LOW check

        if (leftHeld && rightHeld) {
          Serial.println("[SUPERVISOR] Hidden Calibration Sequence Triggered via Multi-Button Hold!");
          
          // Ground light arrays to prevent visual strobe flickering during calibration
          FrontLeft  = false;
          FrontRight = false;
          BackLeft   = false;
          BackRight  = false;

          // Wake MPU sensor registers up so it can cleanly process calibration samples
          if (mpuAvailable) {
            mpu.enableSleep(false);
            vTaskDelay(pdMS_TO_TICKS(50)); // Allow internal I2C bus pipelines to fully stabilize
          }

          // Trigger your existing 6-axis static calibration routine
          // This function automatically flashes all 4 indicators 3 times on success
          if (performSensorCalibration()) {
                        
            // Force your live running filter engines to immediately adopt the fresh RAM calculations
            vehiclePitchRad       = staticMountPitchRad; 
            filteredVehicleAccelY = 0.0;
            
            // Clear out any stale telemetry latch candidates to prevent immediate misfires
            mpuBrakeActive        = false;
            mpuBrakeCandidate     = false;
            mpuEmergencyCandidate = false;
            accumulatedYaw        = 0.0;
            turnInitiated         = false;

            Serial.println("[SUPERVISOR] Manual recalibration sequence completed successfully.");
          } else {
            Serial.println("[SUPERVISOR] Manual recalibration FAILED. Check chasis stability.");
          }
        }
      }

      activeStateProfile = targetStateProfile; // Commit transition tracking state

      switch (activeStateProfile) {
        
        // ==========================================================================
        // STATE 1: FULL OPERATIONAL PROFILE (Ignition ON Regardless of Hazards)
        // ==========================================================================
        case 1:
          Serial.println("[SUPERVISOR] Profile 1: Full System Activation. Resuming all cores...");

          // Wake the physical MPU6050 register infrastructure up if available
          if (mpuAvailable) {
            mpu.enableSleep(false);        // Disable sensor low-power state
            vTaskDelay(pdMS_TO_TICKS(30)); // Allow internal I2C bus pipelines to stabilize
          }

          // Thread Recovery Routine: Un-suspend tasks across both processing cores
          if (telemetryHandle  != NULL) {
            vTaskResume(telemetryHandle);
            esp_task_wdt_add(telemetryHandle); // Re-subscribe to Hardware Watchdog
          }
          if (brakeTaskHandle  != NULL) vTaskResume(brakeTaskHandle);
          if (turnTaskHandle   != NULL) vTaskResume(turnTaskHandle);
          if (inputTaskHandle  != NULL) {
            vTaskResume(inputTaskHandle);
            esp_task_wdt_add(inputTaskHandle); // Re-subscribe to Hardware Watchdog
          }
          if (hazardTaskHandle != NULL) vTaskResume(hazardTaskHandle);
          if (hornTaskHandle   != NULL) vTaskResume(hornTaskHandle);
          if (core0WdtHandle   != NULL) {
            vTaskResume(core0WdtHandle);
            esp_task_wdt_add(core0WdtHandle);  // Re-subscribe Core 0 heartbeat
          }

          Serial.println("[SUPERVISOR] All telemetry, signal, and driver loops fully operational.");
          break;

        // ==========================================================================
        // STATE 2: STATIONARY HAZARD PROFILE (Hazards ON, Ignition OFF)
        // ==========================================================================
        case 2:
          Serial.println("[SUPERVISOR] Profile 2: Ignition Off with Active Hazards. Isolating flasher loops...");

          // Shut down Wi-Fi portal if it was still running when key turned off
          stopWebPortal();

          // Freeze background evaluation engines and telemetry loops to preserve vehicle battery
          if (telemetryHandle != NULL) {
            esp_task_wdt_delete(telemetryHandle);
            vTaskSuspend(telemetryHandle);
          }
          if (brakeTaskHandle  != NULL) vTaskSuspend(brakeTaskHandle);
          if (turnTaskHandle   != NULL) vTaskSuspend(turnTaskHandle);
          if (hornTaskHandle   != NULL) vTaskSuspend(hornTaskHandle);

          // Ensure human interface input loop stays alive on Core 1 to catch the hazard shutoff tap
          if (inputTaskHandle  != NULL) {
            vTaskResume(inputTaskHandle);
            esp_task_wdt_add(inputTaskHandle);
          }
          if (core0WdtHandle   != NULL) {
            vTaskResume(core0WdtHandle);
            esp_task_wdt_add(core0WdtHandle);
          }
          if (hazardTaskHandle != NULL) vTaskResume(hazardTaskHandle);

          // Flush volatile turn and brake variables down to safe default definitions
          signalMode = CMD_TURN_OFF;
          brakeMode  = CMD_BRAKE_OFF;
          
          {
            SignalCommand flushTurn  = CMD_TURN_OFF;
            SignalCommand flushBrake = CMD_BRAKE_OFF;
            xQueueOverwrite(turnQueue,  &flushTurn);
            xQueueOverwrite(brakeQueue, &flushBrake);
          }

          // Drop combination light relays low to prevent background line bleed before hazard override
          FrontLeft  = false;
          FrontRight = false;
          BackLeft   = false;
          BackRight  = false;

          // Put the MPU6050 chip into low-power register standby since tracking is disabled
          if (mpuAvailable) {
            mpu.enableSleep(true); // Halt sensor data recording
            Serial.println("[SUPERVISOR] MPU6050 placed into low-power mode. Hazard engine running isolated.");
          }
          break;

        // ==========================================================================
        // STATE 3: ULTRA-LOW POWER DEEP STANDBY (Hazards OFF, Ignition OFF)
        // ==========================================================================
        case 3:
          Serial.println("[SUPERVISOR] Profile 3: Complete Idle. Commencing deep system shutdown...");

          // 1. Thread Teardown Strategy: Suspend processing loops across both execution cores
          if (telemetryHandle != NULL) {
            esp_task_wdt_delete(telemetryHandle);
            vTaskSuspend(telemetryHandle);
          }
          if (inputTaskHandle != NULL) {
            esp_task_wdt_delete(inputTaskHandle);
            vTaskSuspend(inputTaskHandle);
          }
          if (core0WdtHandle != NULL) {
            esp_task_wdt_delete(core0WdtHandle);
            vTaskSuspend(core0WdtHandle);
          }
          if (brakeTaskHandle  != NULL) vTaskSuspend(brakeTaskHandle);
          if (turnTaskHandle   != NULL) vTaskSuspend(turnTaskHandle);
          if (hazardTaskHandle != NULL) vTaskSuspend(hazardTaskHandle);
          if (hornTaskHandle   != NULL) vTaskSuspend(hornTaskHandle);

          // 2. Hardware Safety Isolation: Ground output channels to protect against continuous battery drain
          for (int i = 0; i < NUM_RELAYS; i++) {
            digitalWrite(relays[i]->value, LOW);
            relays[i]->isOn = false;
          }

          // 3. Sensor Hibernation Block: Place MPU6050 register tracks into sleep mode
          if (mpuAvailable) {
            mpu.enableSleep(true);
            Serial.println("[SUPERVISOR] MPU6050 internal registers placed into standby power levels.");
          }

          // 4. Reset internal software states down to clean base assumptions
          signalMode = CMD_TURN_OFF;
          brakeMode  = CMD_BRAKE_OFF;
          
          {
            SignalCommand cleanTurn  = CMD_TURN_OFF;
            SignalCommand cleanBrake = CMD_BRAKE_OFF;
            xQueueOverwrite(turnQueue,  &cleanTurn);
            xQueueOverwrite(brakeQueue, &cleanBrake);
          }

          Serial.println("[SUPERVISOR] System clean teardown complete. Core 1 entering Light Sleep block...");
          Serial.flush(); // Force serial buffer out before closing execution tracks

          // 5. ESP32 LIGHT SLEEP INTERLOCK PROVISIONING
          // Configure wake anchors: System wakes up immediately on Ignition transition or Hazard switch toggle
          esp_sleep_enable_ext0_wakeup((gpio_num_t)Ignition.value, 0); // Wake when Ignition goes LOW (Keyed ON)
          esp_sleep_enable_ext1_wakeup(1ULL << Hazard.value, ESP_EXT1_WAKEUP_ALL_LOW ); //ESP_EXT1_WAKEUP_ANY_HIGH); // Wake when Hazard switch closes

          // Go into light sleep. Execution stalls here until a wake hardware trigger event fires.
          esp_light_sleep_start();

          // 6. POST-SLEEP RECOVERY INTERLOCK LOGIC
          // Clear sleep parameters cleanly immediately upon recovery to pass authority back to the loop
          esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
          Serial.println("[SUPERVISOR] Core waking up from low power sleep track. Re-evaluating profiles...");
          break;
      }
    }

    /**
    * DYNAMIC SAMPLING CADENCE:
    * When operating normal loops (State 1 or State 2), sample input queues at a balanced 30ms 
    * cadence. Note that State 3 blocks entirely inside the light sleep framework above, 
    * consuming zero CPU cycles until a hardware wake event occurs.
    */
    vTaskDelay(pdMS_TO_TICKS(30));
  }
}
/**
  * @brief High-Water Mark Diagnostics Monitor.
  * Periodically samples the peak stack usage profiles of all running FreeRTOS tasks 
  * on Core 1 to protect against vehicle-induced stack overflows and track memory footprints.
  * 
  * @param pvParameters Standard untyped FreeRTOS task argument block pointer (Unused).
  */
void systemDiagnosticsTask(void* pvParameters) {
  for (;;) {
    // Poll every 5 seconds to prevent spamming the serial bus
    vTaskDelay(pdMS_TO_TICKS(5000));

    Serial.println("\n--- [RTOS MEMORY STACK HIGH-WATER MONITOR] ---");
    Serial.println("Values represent the REMAINING MINIMUM FREE MEMORY in WORDS (4 Bytes).");
    Serial.println("Approaching 0 means risk of stack overflow crash.");

    if (telemetryHandle != NULL) {
      Serial.printf("  -> processVehicleTelemetry : %d words free\n", (int)uxTaskGetStackHighWaterMark(telemetryHandle));
    }
    if (inputTaskHandle != NULL) {
      Serial.printf("  -> mainInputProcessorTask  : %d words free\n", (int)uxTaskGetStackHighWaterMark(inputTaskHandle));
    }
    if (turnTaskHandle != NULL) {
      Serial.printf("  -> turnSignalTask          : %d words free\n", (int)uxTaskGetStackHighWaterMark(turnTaskHandle));
    }
    if (hazardTaskHandle != NULL) {
      Serial.printf("  -> hazardTask              : %d words free\n", (int)uxTaskGetStackHighWaterMark(hazardTaskHandle));
    }
    if (brakeTaskHandle != NULL) {
      Serial.printf("  -> brakeLightTask          : %d words free\n", (int)uxTaskGetStackHighWaterMark(brakeTaskHandle)); // Checked using conditional handle mappings
    }
    if (hornTaskHandle != NULL) {
        Serial.printf("  -> hornTask          : %d words free\n", (int)uxTaskGetStackHighWaterMark(hornTaskHandle)); // Checked using conditional handle mappings
    }

    // Capture the internal supervisor task directly (using a local self-query if called inside it, 
    // but here we can query via its handle if we register one, or monitor remaining system heap)
    Serial.printf("  -> Global Free Static Heap : %d bytes\n", (int)ESP.getFreeHeap());
    Serial.println("-----------------------------------------------\n");
  }
}
/**
  * @brief Lightweight Core 0 heartbeat task subscribed to the Hardware Watchdog.
  *        Verifies Core 0 FreeRTOS scheduler health without waking idle relay tasks.
  * 
  * @param pvParameters Standard untyped FreeRTOS task argument block pointer (Unused).
  */
void core0WatchdogTask(void* pvParameters) {
  esp_task_wdt_add(NULL); // Subscribe this Core 0 task to the TWDT

  for (;;) {
    esp_task_wdt_reset(); // Feed the hardware watchdog
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}