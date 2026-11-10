#include "inputs.h"
#include "web_portal.h"
#include <esp_task_wdt.h>
/**
  * @brief High-speed IRAM-resident Interrupt Service Routine (ISR).
  * Captures clean, edge-triggered mechanical voltage state drops instantly 
  * and passes atomic input changes down to Core 1 tracking registers.
  * 
  * @param arg Untyped raw block parameter containing the target structural Button address.
  */
void IRAM_ATTR buttonISR(void* arg) {
  // Cast raw block parameter back to a structural volatile pointer
  volatile Button* btn = (volatile Button*)arg;
  
  // Convert native 64-bit microsecond counter directly to millisecond base ticks
  uint32_t now = esp_timer_get_time() / 1000;

  // --- 1. HARDWARE SWITCH DEBOUNCE INTERLOCK ---
  // Early exit if the incoming edge falls within the rapid lockout bounce frame window
  if (now - btn->pressStartTime < DEBOUNCE_LOCKOUT) {
    return;
  }

  // --- 2. ATOMIC EDGE BOUNDARY STATE TRACKING ---
  // Evaluate the physical electrical voltage level footprint on the target pin trace
  if (digitalRead(btn->value) == LOW) {
    // Falling Edge Transition: Mechanical contact achieved (Active-LOW switch pulling trace down)
    if (!btn->isPressed) { 
      btn->isPressed      = true;
      btn->pressStartTime = now;
      btn->eventTriggered = false; // Reset trigger tracking engine flags for fresh evaluation loops
    }
  } else {
    // Rising Edge Transition: Mechanical contact released (Pull-up resistor snapping trace high)
    if (btn->isPressed) {
      btn->isPressed      = false;
      btn->pressStartTime = now;
    }
  }
}
/**
  * @brief Processes mechanical brake pedal switch inputs on Core 1 for low-latency override.
  * Evaluates raw hardware rising/falling edge transitions to align software tracking registers 
  * and dispatch synchronized priority state instructions to the Core 0 flasher engine.
  */
void detectBrakePedal() {
  
  // --- 1. RISING-EDGE TRANSITION: Physical human brake pedal strike detected ---
  if (Brake.isPressed && !lastBrakeState) {
    lastBrakeState = true; // Lock edge-detection state tracking latch
    
    // Force background telemetry metrics to align seamlessly with manual human overrides
    mpuBrakeActive    = true; 
    mpuBrakeCandidate = false;

    // EDGE PROTECTION: Suppress queue dispatch if the system profile is already broadcasting active braking
    if (brakeMode != CMD_BRAKE && brakeMode != CMD_EMERGENCY_BRAKE) {
      brakeMode = CMD_BRAKE;
      xQueueOverwrite(brakeQueue, &brakeMode); // Instantly dispatch standard command packet to Core 0
      Serial.println("[MANUAL] Mechanical brake pedal engaged. Sending priority trigger.");
    }
  } 
  
  // --- 2. FALLING-EDGE TRANSITION: Physical human brake pedal release detected ---
  else if (!Brake.isPressed && lastBrakeState) {
    lastBrakeState = false; // Relinquish edge-detection state tracking latch
    
    /**
    * HYSTERESIS COHESION VERIFICATION:
    * Only send a termination instruction if the automated background MPU tracking profile 
    * is not actively enforcing a kinetic deceleration lighting command. If telemetry still 
    * registers heavy g-forces, handoff execution cleanly back to the automated engine.
    */
    if (!mpuBrakeActive) {
      if (brakeMode != CMD_BRAKE_OFF) {
        brakeMode = CMD_BRAKE_OFF;
        xQueueOverwrite(brakeQueue, &brakeMode); // Discharging stop command packet downstream
        Serial.println("[MANUAL] Mechanical brake pedal released. Returning arrays to idle.");
      }
    } else {
      Serial.println("[MANUAL] Pedal released, but dynamic vehicle telemetry maintains active braking.");
    }
  }
}
/**
  * @brief Scans and processes human button interaction layers on Core 1 for turn signals.
  * Evaluates raw hardware edge boundaries to differentiate micro-clicks, mid-range holds, 
  * and persistent latch duration holds to mount multi-tiered expiration timeouts.
  */
void processTurnSignalButtons() {
  uint32_t now = millis();
  
  // Parallel structural translation maps pairing physical addresses to state directives
  Button* turnButtons[2]       = {&Left, &Right};
  SignalCommand triggerCmds[2] = {CMD_LEFT, CMD_RIGHT};

  // Iterate symmetrically over independent signaling channels
  for (int i = 0; i < 2; i++) {
    Button* btn       = turnButtons[i];
    SignalCommand cmd = triggerCmds[i];

    // --- 1. RISING-EDGE DETECTION: Capture initial mechanical switch compression ---
    if (btn->isPressed && !btn->eventTriggered) {
      btn->eventTriggered = true;
      btn->pressStartTime = now;
    }

    // --- 2. FALLING-EDGE DETECTION: Capture structural switch decompression/release ---
    if (!btn->isPressed && btn->eventTriggered) {
      btn->eventTriggered = false; // Clear input latch to accept subsequent transitions
      
      uint32_t holdTime = now - btn->pressStartTime;
      SignalCommand nextSignalMode = signalMode;

      // TRACK OVERRIDE OR TOGGLE DEACTIVATION (MANUAL DOUBLE-PRESS CANCEL)
      // If a driver taps an already active indicator track, interpret it as an abort command
      if (signalMode == cmd) {
        nextSignalMode = CMD_TURN_OFF;
      } 
      
      // FRESH SIGNAL ACTIVATION PATHWAY
      else {
        // PROFILE LEVEL A: Long Hold Profile (Infinite interlock window - e.g., structural breakdown or staging)
        if (holdTime >= HoldDurationInfinite) {
          activeTimeout = 0xFFFFFFFF; 
          Serial.println("[INPUT] Turn signal button persistent long hold registered. Latching infinite timeline.");
        } 
        // PROFILE LEVEL B: Medium Hold Profile (Standard geometric cross-street intersection cornering window)
        else if (holdTime >= HoldDurationTurning) {
          activeTimeout = FlashDurationTurning; 
          Serial.println("[INPUT] Turn signal button intersection hold registered. Latching 20s timeline.");
        } 
        // PROFILE LEVEL C: Short Tap Profile (High-speed vehicle tactical highway lane-change window)
        else {
          activeTimeout = DurationLaneChange; 
          Serial.println("[INPUT] Turn signal button micro-click registered. Latching 5s lane-change timeline.");
        }
        
        nextSignalMode = cmd;
      }

      // --- 3. EDGE-GATED STATE SYNCHRONIZATION ---
      // Only commit modifications and dispatch queue packets if a genuine state alteration occurs
      if (nextSignalMode != signalMode) {
        signalMode = nextSignalMode;
        xQueueOverwrite(turnQueue, &signalMode); // Dispatch master state command packet to Core 0
        
        Serial.print("[INPUT] Turn signal engine state migration committed. Target Mode ID: ");
        Serial.println(signalMode == CMD_TURN_OFF ? "OFF" : (signalMode == CMD_LEFT ? "LEFT" : "RIGHT"));
      }
    }
  }
}
/**
  * @brief Scans and processes human switch inputs on Core 1 for hazard warnings.
  * Evaluates physical toggle latch edge transitions to align software tracking 
  * registers and dispatch synchronized state instructions to the Core 0 hazard engine.
  */
void processHazardButton() {
  // Static edge-detection tracker mapping the physical state of the hardware switch
  static bool lastHazardSwitchState = false;

  // --- 1. EDGE TRANSITION DETECTION LAYER ---
  // Evaluate if the switch has undergone a physical state modification
  if (Hazard.isPressed != lastHazardSwitchState) {
    lastHazardSwitchState = Hazard.isPressed; // Sync the hardware tracking edge latch
    
    SignalCommand nextHazardMode = hazardMode;

    // Map physical switch tracking directly to system state commands
    if (lastHazardSwitchState) {
      nextHazardMode = CMD_HAZARD;
    } else {
      nextHazardMode = CMD_HAZARD_OFF;
    }

    // --- 2. EDGE-GATED STATE SYNCHRONIZATION ---
    // Only execute structural alterations and queue dispatches on genuine changes
    if (nextHazardMode != hazardMode) {
      hazardMode = nextHazardMode;
      xQueueOverwrite(hazardQueue, &hazardMode); // Transmit state instruction across to Core 0
      
      Serial.printf("[INPUT] Physical Hazard Switch transition verified. Core 1 Target State: %s\n", 
                    hazardMode == CMD_HAZARD ? "ACTIVE (CMD_HAZARD)" : "IDLE (CMD_HAZARD_OFF)");
    }
  }
}
/**
  * @brief Scans and processes human interface interaction layers for the vehicle horn.
  * Evaluates mechanical edge transitions and dispatches state updates down the queue backplane.
  */
void processHornButton() {
  static bool lastHornState = false;

  // Active-LOW evaluation: Pressed = true (voltage pulled to ground baseline)
  // Suppress horn if Hazard + Horn portal combo is being held (when ignition interlock is off)
  bool suppressForCombo = (!useIgnitionInterlock && Hazard.isPressed && Horn.isPressed);
  bool currentHornState = suppressForCombo ? false : Horn.isPressed;

  if (currentHornState != lastHornState) {
    lastHornState = currentHornState;
    
    // Explicit Thread-Safe Queue Routing: Dispatch the boolean assert flag downstream
    // Overwrite behavior ensures Core 0 updates instantly without queue overflow lag
    xQueueOverwrite(hornQueue, &lastHornState);
    
    Serial.printf("[INPUT] Mechanical Horn contact edge verified. Dispatched packet state: %s\n",
                  lastHornState ? "ASSERT_HIGH" : "RELEASE_LOW");
  }
}
/**
  * @brief Scans the ignition key state and pushes status directives down
  • to the processSystemInterlockSupervisor queue pipeline on state modifications.
  */
bool processIgnitionButton() {
  static bool lastIgnitionState = false;

  // Active-LOW evaluation: Pressed = LOW (Ignition ON), Unpressed = HIGH (Ignition OFF)
  bool currentIgnitionOn = (digitalRead(Ignition.value) == LOW);
  if (currentIgnitionOn != lastIgnitionState) {
    lastIgnitionState = currentIgnitionOn;
    SignalCommand cmd = currentIgnitionOn ? CMD_IGNITION_ON : CMD_IGNITION_OFF;
    xQueueOverwrite(ignitionQueue, &cmd);
    Serial.printf("[INPUT] Ignition transition caught. Dispatching event to Queue: %s\n",
    currentIgnitionOn ? "CMD_IGNITION_ON" : "CMD_IGNITION_OFF");
  }
  return currentIgnitionOn;
}
/**
  * @brief Monitors for simultaneous Hazard + Horn button hold (2 seconds)
  *        to launch the 5-minute Wi-Fi portal when Ignition Interlock is disabled.
  */
void processPortalComboButton() {
  static uint32_t comboStartTime = 0;
  static bool comboTriggered     = false;

  if (Hazard.isPressed && Horn.isPressed) {
    if (comboStartTime == 0) {
      comboStartTime = millis();
    } else if (!comboTriggered && (millis() - comboStartTime >= 2000)) {
      comboTriggered = true;
      Serial.println("[INPUT] Hazard + Horn hold detected! Launching 5-minute Wi-Fi portal.");
      
      // Launch the 5-minute Wi-Fi portal
      startWebPortal();

      // --- VISUAL FEEDBACK: Flash all 4 corner lights twice ---
      // Temporarily pause Core 0 lighting tasks so they don't fight the confirmation flash
      if (turnTaskHandle   != NULL) vTaskSuspend(turnTaskHandle);
      if (hazardTaskHandle != NULL) vTaskSuspend(hazardTaskHandle);
      if (brakeTaskHandle  != NULL) vTaskSuspend(brakeTaskHandle);

      for (int flash = 0; flash < 2; flash++) {
        FrontLeft  = true;
        FrontRight = true;
        BackLeft   = true;
        BackRight  = true;
        vTaskDelay(pdMS_TO_TICKS(70));

        FrontLeft  = false;
        FrontRight = false;
        BackLeft   = false;
        BackRight  = false;
        vTaskDelay(pdMS_TO_TICKS(70));
      }

      // Resume Core 0 lighting tasks cleanly
      if (turnTaskHandle   != NULL) vTaskResume(turnTaskHandle);
      if (hazardTaskHandle != NULL) vTaskResume(hazardTaskHandle);
      if (brakeTaskHandle  != NULL) vTaskResume(brakeTaskHandle);
    }
  } else {
    // Reset tracking once either button is released
    comboStartTime = 0;
    comboTriggered = false;
  }
}
/**
  • @brief Dedicated Human Interface Scanning Task isolated to Core 1.
  * 
  * @param pvParameters Standard untyped FreeRTOS task argument block pointer (Unused).
  */
void mainInputProcessorTask(void* pvParameters) {
  esp_task_wdt_add(NULL); // Subscribe Core 1 Input Task to Hardware Watchdog
  uint32_t waitTime = 15;
  bool ignitionOn = false;
  for (;;) {
    // Feed watchdog on every input scan cycle
    esp_task_wdt_reset();
    // Standard Wire Check: Only query the physical pin if the interlock wire is used
    if (useIgnitionInterlock){
      ignitionOn = processIgnitionButton();
    } else {
      // Allow manual portal launch via Hazard + Horn hold when ignition wire is not used
      processPortalComboButton();
    }
    // Core Execution Gate: Allow inputs if ignition is physically on, OR if interlock is disabled entirely
    if (ignitionOn || !useIgnitionInterlock) {
      if (useBrakelights) {
        detectBrakePedal();
      }
      processTurnSignalButtons();
      processHornButton(); 
    }
    processHazardButton();

    // Core Execution Gate: If interlock is disabled, we proceed immediately as if
    // ignition is active, bypassing all key-wire checks for standalone operation.
    if (ignitionOn || hazardMode == CMD_HAZARD || !useIgnitionInterlock) {
      waitTime = 15;
    } else {
      waitTime = 200; 
    }

    vTaskDelay(pdMS_TO_TICKS(waitTime));
  }
}