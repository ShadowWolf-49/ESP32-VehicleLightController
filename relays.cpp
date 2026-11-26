#include "relays.h"
#include <esp_task_wdt.h>

// Runs automatically inside the WDT interrupt right before panic/reboot
extern "C" void IRAM_ATTR esp_task_wdt_isr_user_handler(void) {
  groundAllRelaysFailSafe();
}

/**
  * @brief Immediately grounds all physical relay outputs to a safe LOW state.
  */
void IRAM_ATTR groundAllRelaysFailSafe() {
  for (int i = 0; i < NUM_RELAYS; i++) {
    gpio_set_level((gpio_num_t)relays[i]->value, 0);
    relays[i]->isOn = false;
  }
}
/**
  * @brief Background task running on Core 0 to handle brake relay actuation.
  * Processes manual and telemetry brake commands asynchronously, coordinates 
  * with turn signals and hazard tasks, and enforces strict output line priorities.
  * 
  * @param pvParameters Standard untyped FreeRTOS task argument block pointer (Unused).
  */
void brakeLightTask(void* pvParameters) {
  // --- Local Task State Storage ---
  SignalCommand receivedCmd       = CMD_BRAKE_OFF; // Initialize to safe default
  SignalCommand currentSignalMode = CMD_TURN_OFF;  // Turn signal state snapshot
  SignalCommand currentHazardMode = CMD_HAZARD_OFF; // Hazard state snapshot
  
  uint32_t flashTimer      = 0;
  uint32_t duration        = 1000;
  uint32_t interval        = 140;
  bool strobeState         = false;
  bool durationComplete    = false;
  bool brakeActive         = false;

  // Infinite execution boundary container keeping Core 0 background OS scheduler thread fed
  for (;;) {
    // Temporary holder to check incoming message data safely without destroying running clocks
    SignalCommand incomingCmd;

    /**
    * DYNAMIC QUEUE BLOCKING STRATEGY:
    * - When IDLE (!brakeActive), block indefinitely (portMAX_DELAY) to reduce CPU load to 0%.
    * - When ACTIVE (brakeActive), poll instantly (0-timeout) to allow flasher clocks to cycle.
    */
    uint32_t waitTime = (!brakeActive && currentHazardMode != CMD_HAZARD) ? portMAX_DELAY : 0;

    // 1. INBOUND QUEUE DATA ACQUISITION
    if (xQueueReceive(brakeQueue, &incomingCmd, waitTime) == pdPASS) { 
      // Synchronize latest structural state changes across to Core 1 instantly
      xQueueOverwrite(brakeToTurnQueue, &incomingCmd);
      
      /**
      * STATE EDGE TRANSITION GUARD:
      * Only reset the running millisecond baselines if a brand-new distinct command arrives,
      * or if transitioning out of an idle state. This isolates the timing engine from 
      * continuous xQueueOverwrite streams originating on Core 1 when a pedal or MPU is held.
      */
      if (incomingCmd != receivedCmd || !brakeActive) {
        brakeStartTime   = millis();
        flashTimer       = millis();
        strobeState      = true;
        durationComplete = false;
      }
      
      receivedCmd = incomingCmd; // Safe commit to master state register
    }

    // High-Speed Status Sync: Non-blocking fetch of neighbor task statuses
    xQueueReceive(turnToBrakeQueue, &currentSignalMode, 0);
    xQueueReceive(hazardToBrakeQueue, &currentHazardMode, 0);

    // --- 2. HIGH-PRIORITY HAZARD INTERLOCK OVERRIDE CHECK ---
    if (currentHazardMode == CMD_HAZARD) {
      // If hazard system is actively interlocking outputs, force this task's internal
      // tracking loop flags active to track the current braking status behind the scenes,
      // but bypass direct relay control so hazardTask can manage blended combination logic.
      brakeActive = (receivedCmd == CMD_BRAKE || receivedCmd == CMD_EMERGENCY_BRAKE);
      
      // Update distribution pipes for hazard engine awareness before yielding core execution
      xQueueOverwrite(brakeToHazardQueue, &receivedCmd);
      vTaskDelay(pdMS_TO_TICKS(10));
      continue; 
    }

    // --- 3. STATE MACHINE ROUTING LAYER ---
    switch (receivedCmd) {
      case CMD_BRAKE:
      case CMD_EMERGENCY_BRAKE:
        brakeActive = true;
        
        // Dynamically assign parameters based on specific braking severity command profiles
        if (receivedCmd == CMD_BRAKE) {
          duration = brakeFlashDuration;
          interval = brakeFlashInterval;
        } else if (receivedCmd == CMD_EMERGENCY_BRAKE) {
          duration = EmergencyFlashDuration;
          interval = EmergencyFlashInterval;
        }
        
        // --- 4. UNIFIED ASYNCHRONOUS FLASHER ENGINE ---
        if (!durationComplete) {
          // Check if the flash duration countdown window has completed
          if (millis() - brakeStartTime >= duration) {
            durationComplete = true;
            // Transition back to solid structural illumination, respecting running turn signal vectors
            if (currentSignalMode != CMD_LEFT)  BackLeft = true;
            if (currentSignalMode != CMD_RIGHT) BackRight = true;
          } 
          // Check if dynamic flasher cadence toggle interval has elapsed
          else if (millis() - flashTimer >= interval) {
            flashTimer  = millis();
            strobeState = !strobeState;
            
            // Apply strobe state modifications while bypassing active turn signal channels
            if (currentSignalMode != CMD_LEFT)  BackLeft = strobeState;
            if (currentSignalMode != CMD_RIGHT) BackRight = strobeState;
          } else {
            // Ensure non-turning rear channel stays synced with current strobe phase
            // immediately after a turn signal cancels mid-brake
            if (currentSignalMode != CMD_LEFT)  BackLeft = strobeState;
            if (currentSignalMode != CMD_RIGHT) BackRight = strobeState;
          }
        } else {
          // Maintain continuous baseline safety hold state after strobe window expires
          if (currentSignalMode != CMD_LEFT)  BackLeft = true;
          if (currentSignalMode != CMD_RIGHT) BackRight = true;
        }
        break;

      case CMD_BRAKE_OFF:
      default:
        // Safe Idle Handler Profile: Clear dynamic states
        brakeActive = false;
        
        // Only drop combination elements low if they are not actively assigned to a turning task
        if (currentSignalMode != CMD_LEFT)  BackLeft = false;
        if (currentSignalMode != CMD_RIGHT) BackRight = false;
        break;
    }

    // --- 5. DATA SYNCHRONIZATION BACKPLANE DISTRIBUTION ---
    // Overwrite output pipeline mailboxes to keep turn and hazard engines tightly in sync
    xQueueOverwrite(brakeToTurnQueue, &receivedCmd);
    xQueueOverwrite(brakeToHazardQueue, &receivedCmd);

    // Cooperative Concurrency Yield: Mandatory sleep to feed system hardware watchdogs on Core 0
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
/**
  * @brief Background task running on Core 0 to handle turn signal relay actuation.
  * Processes turn commands asynchronously, coordinates with brake and hazard tasks,
  * manages multi-tiered timeouts, and enforces strict output line priorities.
  * 
  * @param pvParameters Standard untyped FreeRTOS task argument block pointer (Unused).
  */
void turnSignalTask(void* pvParameters) {
  // --- Local Task State Storage ---
  SignalCommand receivedCmd       = CMD_TURN_OFF;   // Initialize to safe default
  SignalCommand currentBrakeMode  = CMD_BRAKE_OFF;  // Background brake tracking status
  SignalCommand currentHazardMode = CMD_HAZARD_OFF; // Background hazard tracking status

  volatile uint32_t dynamicModeStartTime = 0;       // Baseline execution timestamp defining signal auto-kill windows
  
  uint32_t flashTimer  = 0;     // High-resolution timestamp tracking individual blinker cadences
  bool blinkToggle     = false; // Local asynchronous logic level cache for physical relay pins
  bool turnActive      = false; // Internal tracking flag dictating dynamic queue blocking timeouts
  bool wasTurnActive   = false; // <-- ADDED: Tracks previous active turn state for clean single-pass handoff

  // Infinite execution boundary container keeping Core 0 background OS scheduler thread fed
  for (;;) {
    // Temporary holder to handle incoming messages without breaking clock baselines
    SignalCommand incomingCmd;

    /**
    * DYNAMIC QUEUE BLOCKING STRATEGY:
    * - When IDLE (!turnActive and hazards off), block indefinitely (portMAX_DELAY) to drop Core 0 usage to 0%.
    * - When ACTIVE, poll instantly (0-timeout) to allow the flasher clocks to run.
    */
    uint32_t waitTime = (!turnActive && currentHazardMode != CMD_HAZARD) ? portMAX_DELAY : 0;

    // 1. INBOUND QUEUE DATA ACQUISITION
    if (xQueueReceive(turnQueue, &incomingCmd, waitTime) == pdPASS) {
      // Synchronize latest structural state changes across to Core 1 instantly
      xQueueOverwrite(turnToBrakeQueue, &incomingCmd);
      
      /**
      * STATE EDGE TRANSITION GUARD:
      * Only reset the running millisecond baselines if a brand-new distinct command arrives,
      * or if transitioning out of an idle state. This isolates the timing engine from 
      * continuous xQueueOverwrite streams originating on Core 1 when a button is held.
      */
      if (incomingCmd != receivedCmd || !turnActive) {
        dynamicModeStartTime = millis(); // Initialize the dynamic auto-kill lifetime countdown window
        flashTimer           = millis(); // Reset cadence timer registers to begin flash cycles cleanly
        blinkToggle          = true;     // Turn indicator traces HIGH instantly on initial edge strike
      }

      receivedCmd = incomingCmd; // Safe commit to master state register
    }

    // High-Speed Status Sync: Non-blocking fetch of neighbor task statuses
    xQueueReceive(brakeToTurnQueue, &currentBrakeMode, 0);
    xQueueReceive(hazardToTurnQueue, &currentHazardMode, 0);

    // Update internal state variable for timing tracking configurations
    turnActive = (receivedCmd == CMD_LEFT || receivedCmd == CMD_RIGHT);

    // Broadcast current turn state out to the hazard backplane immediately
    xQueueOverwrite(turnToHazardQueue, &receivedCmd);

    // --- 2. HIGH-PRIORITY HAZARD INTERLOCK OVERRIDE CHECK ---
    if (currentHazardMode == CMD_HAZARD) {
      // If hazard system is active, force turn signal processing to stand down.
      // Bypass direct relay control so hazardTask can manage all four elements.
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // <-- MOVED & ADDED: Compute brakeActive here so all branches below can read it
    bool brakeActive = (currentBrakeMode == CMD_BRAKE || currentBrakeMode == CMD_EMERGENCY_BRAKE || Brake.isPressed);
    
    // --- 3. ACTIVE ASYNCHRONOUS FLASHER ENGINE ---
    if (turnActive) {
      // Latch state so idle handoff runs once when turn finishes
      wasTurnActive = true; 

      // Compute total elapsed duration since initial channel engagement
      uint32_t duration = millis() - dynamicModeStartTime;

      // CRITICAL TIMEOUT TRACKER: Auto-terminate operation if the current time window exceeds active limits
      if (duration >= activeTimeout) {
        receivedCmd = CMD_TURN_OFF; // Force tracking registers back to idle states
        turnActive  = false;
        wasTurnActive = false;
        
        // Immediate clean teardown profile
        FrontLeft   = false;               
        FrontRight  = false;
        
        // RECOVERY ROUTINE: Hand off the rear combination outputs back to their expected 
        // background braking baseline exclusively when the blinking sequence terminates.
        BackLeft   = brakeActive;
        BackRight  = brakeActive;

        // Notify Core 1 that the turn signal timed out
        SignalCommand cancelCmd = CMD_TURN_OFF;
        xQueueOverwrite(turnCancelQueue, &cancelCmd);
      } else {
        // TIMING CADENCE SWITCH: Toggle physical trace footprints once interval parameters are achieved
        if (millis() - flashTimer >= blinkFlashInterval) {
          flashTimer  = millis();      // Advance differential reference frame
          blinkToggle = !blinkToggle;  // Alternate internal logic state
        }

        // Apply active turn outputs immediately on edge strike and every cadence tick
        if (receivedCmd == CMD_LEFT) {
          FrontLeft  = blinkToggle;   
          BackLeft   = blinkToggle;   // Left turning track takes absolute priority over brakes
          FrontRight = false;         // Double-guard isolation: Maintain opposing track low
          BackRight  = brakeActive;   // Opposing side rear element maintains standard brake behavior
        } else if (receivedCmd == CMD_RIGHT) {
          FrontLeft  = false;         // Double-guard isolation: Maintain opposing track low
          BackLeft   = brakeActive;   // Opposing side rear element maintains standard brake behavior
          FrontRight = blinkToggle;  
          BackRight  = blinkToggle;   // Right turning track takes absolute priority over brakes
        }
      }
    } 
    
    // --- 4. SAFE IDLE HANDOFF HANDLING LAYER (Fired when receivedCmd == CMD_TURN_OFF) ---
    else if (wasTurnActive) {
      wasTurnActive = false;
      // Clean flush: Instantly drop front vehicle indicator lines to a low potential trace state
      FrontLeft  = false;
      FrontRight = false;
      
      // Hand rear channels back to baseline braking state once; brakeLightTask owns them while idle
      BackLeft   = brakeActive;
      BackRight  = brakeActive;
    }

    // Direct status reflect back downstream to the brake light task thread-safe
    xQueueOverwrite(turnToBrakeQueue, &receivedCmd);

    // Cooperative Concurrency Yield: Mandatory sleep to feed system hardware watchdogs on Core 0
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
/**
  * @brief Background task running on Core 0 to handle hazard warning relay actuation.
  * Manages concurrent blinker masking, applies stroboscopic brake blending equations, 
  * and handles emergency priority interlock routing when combined with human pedal inputs.
  * 
  * @param pvParameters Standard untyped FreeRTOS task argument block pointer (Unused).
  */
void hazardTask(void* pvParameters) {
  SignalCommand receivedHazardCmd = CMD_HAZARD_OFF;
  SignalCommand currentBrakeMode  = CMD_BRAKE_OFF;
  SignalCommand currentTurnMode   = CMD_TURN_OFF;

  uint32_t flashTimer   = 0;
  uint32_t cycleTimer   = 0;
  bool hazardToggle     = false;
  bool hazardActive     = false;
  bool wasHazardActive  = false;

  for (;;) {
    SignalCommand incomingCmd;
    uint32_t waitTime = (!hazardActive) ? portMAX_DELAY : 0;

    // 1. DYNAMIC BLOCKING QUEUE PARSER
    if (xQueueReceive(hazardQueue, &incomingCmd, waitTime) == pdPASS) {
      if (incomingCmd != receivedHazardCmd || !hazardActive) {
        flashTimer   = millis();
        cycleTimer   = millis();
        hazardToggle = true; // Instantly illuminate elements on initial edge strike
      }
      receivedHazardCmd = incomingCmd;
    }

    // 2. INTER-TASK PIPELINE DATA ACQUISITION (Drains latest snapshots non-blockingly)
    xQueueReceive(brakeToHazardQueue, &currentBrakeMode, 0);
    xQueueReceive(turnToHazardQueue, &currentTurnMode, 0);
    
    hazardActive = (receivedHazardCmd == CMD_HAZARD);

    // --- 3. ACTIVE HAZARD ENGINE LAYER ---
    if (hazardActive) {
      wasHazardActive = true;
      uint32_t now = millis();

      // Half-period toggle for front hazard lights
      if (now - flashTimer >= hazardFlashInterval) {
        flashTimer   = now;
        hazardToggle = !hazardToggle;
      }

      // Full-period cycle timer (ON + OFF = 2 * hazardFlashInterval) for brake-blended duty cycle
      uint32_t fullCyclePeriod = hazardFlashInterval * 2;
      if (now - cycleTimer >= fullCyclePeriod) {
        cycleTimer = now;
      }

      bool isBraking = (currentBrakeMode == CMD_BRAKE || currentBrakeMode == CMD_EMERGENCY_BRAKE);

      if (isBraking) {
        /**
        * MATHEMATICAL DUTY CYCLE BLENDING ROUTINE:
        * Calculates if the running clock falls inside the solid-high window 
        * or the blinker-low release window based on brakeHazardPercent configuration maps.
        */
        uint32_t currentPosition = (now - cycleTimer);
        uint32_t highWindowLimit = (uint32_t)((fullCyclePeriod * brakeHazardPercent) / 100.0f);
        bool brakeDutyCycleHigh  = (currentPosition < highWindowLimit);

        if (brakeDutyCycleHigh) {
          BackLeft  = true;
          BackRight = true;
        } else {
          BackLeft  = false;
          BackRight =false;
        }
        
        // Front markers stay isolated to clean hazard flasher oscillations
        FrontLeft  = hazardToggle;
        FrontRight = hazardToggle;

      } else {
        // Normal Operation: Hazards run symmetrically on all four corner pins
        FrontLeft  = hazardToggle;
        FrontRight = hazardToggle;
        BackLeft   = hazardToggle;
        BackRight  = hazardToggle;
      }
    }

    // --- 3B. CLEAN HANDOFF / RELEASE LAYER WHEN HAZARDS TURN OFF ---
    else if (wasHazardActive) { // <-- FIXED: Only run cleanup once on falling edge so idle loop doesn't fight other tasks
      wasHazardActive = false;

      // Drop front markers if no turn signal is actively claiming them
      if (currentTurnMode != CMD_LEFT)  FrontLeft  = false;
      if (currentTurnMode != CMD_RIGHT) FrontRight = false;

      // Drop rear markers if neither a turn signal nor brake is actively claiming them
      bool isBraking = (currentBrakeMode == CMD_BRAKE || currentBrakeMode == CMD_EMERGENCY_BRAKE);
      if (!isBraking) {
        if (currentTurnMode != CMD_LEFT)  BackLeft  = false;
        if (currentTurnMode != CMD_RIGHT) BackRight = false;
      }
    }
    
    // --- 4. DATA SYNCHRONIZATION BACKPLANE DISTRIBUTION ---
    // Overwrite shared loop registers so Brake and Turn engines yield hardware priority gracefully
    xQueueOverwrite(hazardToBrakeQueue, &receivedHazardCmd);
    xQueueOverwrite(hazardToTurnQueue, &receivedHazardCmd);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
/**
  * @brief Background task running on Core 0 to handle direct horn relay actuation.
  * Processes human interaction updates asynchronously from the FreeRTOS stream pipe.
  * 
  * @param pvParameters Standard untyped FreeRTOS task argument block pointer (Unused).
  */
void hornTask(void* pvParameters) {
  // Local task state tracker matching your safe execution profile philosophy
  bool assertHorn = false;

  for (;;) {
    /**
    * DYNAMIC QUEUE BLOCKING STRATEGY:
    *  - When IDLE (!assertHorn), block indefinitely (portMAX_DELAY) to drop Core 0 CPU load to 0%.
    *  - When ACTIVE (assertHorn), poll instantly (0-timeout) to process immediate releases.
    */
    uint32_t waitTime = (!assertHorn) ? portMAX_DELAY : 0;

    // Drain the latest command token from the pipeline
    if (xQueueReceive(hornQueue, &assertHorn, waitTime) == pdPASS) {
      // Execute low-impedance relay configuration instantly using your structural operator
      HornRelay = assertHorn;
    }

    // Cooperative Concurrency Yield: Mandatory sleep to feed system hardware watchdogs on Core 0
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}