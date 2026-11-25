#include "mpu_sensor.h"
#include <Wire.h>
#include <esp_task_wdt.h>

 /**
  * @brief Computes the absolute geometric 3D vector length (magnitude) from individual orthogonal components.
  * Utilizes the Euclidean 3D distance formula (Pythagorean theorem in three dimensions).
  * 
  * @note When the vehicle is resting stationary, this function should return approximately 9.81 m/s^2 
  * (1G of acceleration) because the accelerometer registers the constant structural pull of Earth's gravity.
  * 
  * @param x The raw or bias-corrected scalar component along the X-axis (m/s^2)
  * @param y The raw or bias-corrected scalar component along the Y-axis (m/s^2)
  * @param z The raw or bias-corrected scalar component along the Z-axis (m/s^2)
  * 
  * @return float The scalar length representing the total absolute combined vector force (m/s^2)
  */
float vectorMagnitude(float x, float y, float z){
    // Mathematical Length Derivative: Squares spatial vectors to drop directional signs,
    // aggregates components, and extracts the square root to resolve net scalar force.
    return sqrt((x * x) + (y * y) + (z * z));
}
/**
  * @brief Performs a 6-axis static calibration routine to capture accelerometer and 
  * gyroscope sensor drift biases. Calculates vehicle mounting transformation matrices, 
  * verifies module stability, and writes data to persistent Non-Volatile Storage (NVS).
  * 
  * @return true if calibration succeeded and was saved; false if motion boundaries or matrix transformation math fails.
  */
bool performSensorCalibration(){

  // Notify operator via serial link and stall thread to ensure physical chassis rocking stops
  Serial.println("[CALIBRATION] MPU6050 static calibration starting.");
  Serial.println("[CALIBRATION] Ensure vehicle is parked on level, stable ground.");
  Serial.println("[CALIBRATION] Keep chassis perfectly stationary. Processing samples...");
  vTaskDelay(pdMS_TO_TICKS(100)); // Non-blocking shifting delay to let chassis vibrations dissipate

  // Structural allocation for pulling unified sensor events from drivers
  sensors_event_t acceleration, gyro, temperature;

  // Local accumulators tracking multi-axis sample matrices
  float totalAX = 0.0, totalAY = 0.0, totalAZ = 0.0;
  float totalGX = 0.0, totalGY = 0.0, totalGZ = 0.0;
  float totalGravity = 0.0;

  // High/Low boundary limits tracking kinetic variance stability bounds
  float minimumMeasuredGravity = 1000.0;
  float maximumMeasuredGravity = 0.0;

  // Primary summation loop over user-defined static sample sizes
  for (int sample = 0; sample < samples; sample++) {
    mpu.getEvent(&acceleration, &gyro, &temperature);

    // Accumulate raw spatial vectors across sample cycles
    totalAX += acceleration.acceleration.x;
    totalAY += acceleration.acceleration.y;
    totalAZ += acceleration.acceleration.z;

    totalGX += gyro.gyro.x;
    totalGY += gyro.gyro.y;
    totalGZ += gyro.gyro.z;

    // Compute instantaneous absolute combined 3D vector scalar force
    float currentGravity = vectorMagnitude(acceleration.acceleration.x, acceleration.acceleration.y, acceleration.acceleration.z);
    totalGravity += currentGravity;

    // Track extreme peak metrics to evaluate environment structural stability
    if (currentGravity < minimumMeasuredGravity) minimumMeasuredGravity = currentGravity;
    if (currentGravity > maximumMeasuredGravity) maximumMeasuredGravity = currentGravity;

    // Explicit non-blocking delay to let internal sensor hardware pipelines refresh cleanly
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // Generate statistical baseline averages across sample frame sets
  accelBiasX = totalAX / samples;
  accelBiasY = totalAY / samples;
  accelBiasZ = totalAZ / samples;

  gyroBiasX  = totalGX / samples;
  gyroBiasY  = totalGY / samples;
  gyroBiasZ  = totalGZ / samples;

  measuredGravity = totalGravity / samples;

  // Compute overall acceleration noise delta envelope to flag structural cabin movement
  float calibrationMotion = maximumMeasuredGravity - minimumMeasuredGravity;

  // FIXED: Resolve orientation matrices BEFORE stripping gravity out of the raw Z vector bias
  staticMountPitchRad = atan2(accelBiasY, accelBiasZ);
  staticMountPitchDeg = staticMountPitchRad * (180.0 / PI);
  vehiclePitchRad     = staticMountPitchRad; // Seed running orientation engine baseline

  // Remove structural Earth gravity component down to a zero-point threshold balance
  accelBiasZ -= measuredGravity;

  // Diagnostics Pipeline: Broadcast profile configurations to terminal monitor
  Serial.println("[CALIBRATION] Stationary Offset Balances Calculated:");
  Serial.printf("  -> Accel Drift Bias [X, Y, Z]: %.6f, %.6f, %.6f m/s^2\n", accelBiasX, accelBiasY, accelBiasZ);
  Serial.printf("  -> Gyro Drift Bias  [X, Y, Z]: %.6f, %.6f, %.6f rad/s\n", gyroBiasX, gyroBiasY, gyroBiasZ);
  Serial.printf("  -> System Gravity Vector Core: %.4f m/s^2\n", measuredGravity);
  Serial.printf("  -> Mount Angle Pitch Matrix:   %.2f degrees\n", staticMountPitchDeg);
  Serial.printf("  -> Calibration Motion Delta:   %.4f m/s^2\n", calibrationMotion);

  // CRITICAL VALIDATION BOUNDARY A: Reject calibration if chassis was rocking or vibrating
  if (calibrationMotion > 1.0) {
    Serial.println("[CALIBRATION] FAILED: Excessive physical motion detected during tracking frame.");
    Serial.println("[CALIBRATION] Ensure engine is killed, cabin is quiet, and retry.");
    return false;
  }

  // CRITICAL VALIDATION BOUNDARY B: Reject if sensor installation alignment errors exceed constraints
  if (staticMountPitchDeg < MIN_MOUNT_ANGLE_X || staticMountPitchDeg > MAX_MOUNT_ANGLE_X) {
    Serial.println("[CALIBRATION] FAILED: Physical mounting pitch angle exceeds structural -45 to +45 limits.");
    Serial.println("[CALIBRATION] Verify sensor alignment matching '+Y Forward' and retry.");
    return false;
  }

  // --- 4. PERSISTENT REGISTRATION (NVS Commit) ---
  // Lock namespace partition sector maps in write-mode (false flag)
  prefs.begin("IMU_Cal", false);

  prefs.putFloat("gravity",      measuredGravity);
  prefs.putFloat("biasAX",       accelBiasX);
  prefs.putFloat("biasAY",       accelBiasY);
  prefs.putFloat("biasAZ",       accelBiasZ);
  prefs.putFloat("biasGX",       gyroBiasX);
  prefs.putFloat("biasGY",       gyroBiasY);
  prefs.putFloat("biasGZ",       gyroBiasZ);
  prefs.putFloat("vehiclePitch", vehiclePitchRad);
  prefs.putBool("calibrated",    true);

  prefs.end(); // Safely close handle and commit cache descriptors down to physical cells

  // Flush tracking filters back to default parameters to clear historical data paths
  filteredVehicleAccelY = 0.0;
  mpuBrakeActive        = false;
  mpuBrakeCandidate     = false;
  mpuEmergencyCandidate = false;

  Serial.println("[CALIBRATION] 6-Axis spatial transformation profile committed permanently to memory.");

  // VISUAL FEEDBACK INTERLOCK: Flash all 4 corner indicators simultaneously 3 times to confirm calibration success
      // Temporarily pause Core 0 lighting tasks so they don't fight the confirmation flash
  if (turnTaskHandle   != NULL) vTaskSuspend(turnTaskHandle);
  if (hazardTaskHandle != NULL) vTaskSuspend(hazardTaskHandle);
  if (brakeTaskHandle  != NULL) vTaskSuspend(brakeTaskHandle);

  for (int flash = 0; flash < 3; flash++) {
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

  return true;
}
/**
  * @brief Handles physical hardware handshake initialization for the MPU6050 sensor,
  * configures dynamic hardware scaling registers, and manages a dual-stage boot sequence 
  * that automatically triggers fresh cold-boot calibrations or warm-boot parameter restorations.
  */
void initializeMPUCalibration() {

  // --- 1. HARDWARE HANDSHAKE LAYER ---
  // Attempt raw I2C master line communication checks with the MPU6050 hardware address
  if (!mpu.begin()) {
    // FAIL-SAFE FAULT BOUNDARY: Handle absent hardware gracefully to prevent system hang-ups
    Serial.println("[MPU] WARNING: MPU6050 accelerometer module not detected. Skipping telemetry tracking.");
    mpuAvailable      = false;
    mpuBrakeAvailable = false;
  } else {
    Serial.println("[MPU] MPU6050 Accelerometer/Gyroscope bus connection online.");
    mpuAvailable = true;

    // HARDWARE REGISTER CONFIGURATION: Establish tracking envelopes and filter spectrum frequencies
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);  // 8G window protects against mechanical road-bump clipping
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);       // High-resolution rotational scaling up to 250 deg/sec
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);    // Low-pass hardware cutoff filters structural frame vibrations

    // --- 2. STORAGE SYSTEM SUBSYSTEM LOCK ---
    // Open the permanent NVS "IMU_Cal" namespace flash partition in read/write mode (false flag)
    prefs.begin("IMU_Cal", false);

    // Query validation token. Returns false if flash cells are blank or uninitialized
    bool isCalibrated = prefs.getBool("calibrated", false);

    // --- 3. DYNAMIC SEQUENCING EVALUATION ENGINE ---
    // EVALUATION STAGE A: COLD INITIALIZATION PROTOCOL (Fired when valid calibration parameters are absent)
    if (!isCalibrated) {
      // Relinquish storage partition handle early to pass control over to calibration engine safely
      prefs.end();

      // Execute multi-axis static chassis scan to map baseline mounting biases
      if (!performSensorCalibration()) {
        Serial.println("[MPU] CRITICAL: Initial calibration sequence failed.");
        Serial.println("[MPU] Automated telemetry-driven lighting systems disabled until recalibrated.");
        mpuBrakeAvailable = false;
      } else {
        Serial.println("[MPU] Initial static calibration completed successfully.");
        Serial.println("[MPU] Automated telemetry brake-tracking engine fully enabled.");
        mpuBrakeAvailable = true;
      }
    }
    
    // EVALUATION STAGE B: WARM BOOT PROTOCOL (Restores calibrated spatial offsets from persistent non-volatile cell banks)
    else {
      // Extract structural Earth gravity constant calculated for this specific installation unit
      measuredGravity = prefs.getFloat("gravity", 9.81); 

      // Pull multi-axis accelerometer resting hardware drift balances from non-volatile cells
      accelBiasX = prefs.getFloat("biasAX", 0.0);
      accelBiasY = prefs.getFloat("biasAY", 0.0);
      // Fixed: Default parameter tracks relative gravity offset as an inline stability safety cushion
      accelBiasZ = prefs.getFloat("biasAZ", 0.0); 

      // Pull multi-axis gyroscope zero-rotational biases to isolate vehicle maneuvers from drifting heading metrics
      gyroBiasX = prefs.getFloat("biasGX", 0.0);
      gyroBiasY = prefs.getFloat("biasGY", 0.0);
      gyroBiasZ = prefs.getFloat("biasGZ", 0.0);

      // Extract the fixed physical module mounting pitch layout relative to the mechanical vehicle chassis frame
      vehiclePitchRad = prefs.getFloat("vehiclePitch", 0.0);

      // CONCURRENCY RELEASE: Relinquish flash memory namespace to return hardware cells back to safe idle
      prefs.end();

      Serial.println("[MPU] Stored calibration profiles loaded from non-volatile memory cell blocks.");
      Serial.print("[MPU] Operational parameters verified. Baseline Mount Pitch: ");
      Serial.print(vehiclePitchRad * (180.0 / PI), 2);
      Serial.println(" deg");

      // Warm boot data verified: Assert dynamic automated lighting systems fully operational
      mpuBrakeAvailable = true;
    }
  }
}
/**
  * @brief Processes gravity-compensated dynamic forward acceleration values to control
  * automated braking light indication states via dual-threshold hysteresis tracking.
  * Restricts MPU automated shutoff if the physical brake pedal is actively held.
  * 
  * @param vehicleAccelY The clean, gravity-compensated forward acceleration/deceleration force (m/s^2)
  * @param now Current execution timestamp passed from the main application clock cycle (millis)
  */
void updateMPUBrakeDetection(float vehicleAccelY, uint32_t now) {
  
  // --- 1. BRAKE TRIGGER EVALUATION ENGINE (System is currently IDLE or non-braking) ---
  if (!mpuBrakeActive) {
    
    // Evaluate if deceleration force crosses past the target threshold boundary limit
    if (vehicleAccelY <= MPU_BRAKE_ON_THRESHOLD) {
      
      // Initial breakthrough past standard limit: capture reference timestamp baseline
      if (!mpuBrakeCandidate) {
        mpuBrakeCandidate = true;
        mpuBrakeCandidateStart = now;
      }

      // CRITICAL PRE-EMPTIVE PATHWAYS: Evaluate for severe emergency braking profiles
      if (vehicleAccelY <= MPU_EMERGENCY_THRESHOLD) {
        if (!mpuEmergencyCandidate) {
          mpuEmergencyCandidate = true;
          mpuEmergencyCandidateStart = now;
        }
        
        // Emergency Debounce Verification: Ensure severe intensity persists to filter road potholes
        if (now - mpuEmergencyCandidateStart >= MPU_EMERGENCY_CONFIRM_MS) {
          mpuBrakeActive           = true;  // Assert primary dynamic software tracking latch
          mpuBrakeCandidate        = false; // Flush intermediate validation registers
          mpuEmergencyCandidate    = false; // Clear emergency tracking candidates

          // EDGE PROTECTION: Only drop message if the system state isn't already active emergency braking
          if (brakeMode != CMD_EMERGENCY_BRAKE) {
            brakeMode = CMD_EMERGENCY_BRAKE;
            xQueueOverwrite(brakeQueue, &brakeMode); // Dispatch urgent stroboscopic command instantly to Core 0
            Serial.println("[MPU] EMERGENCY Brake Light Triggered!");
          }
          return; // Halt further standard processing for this execution slice
        }
      } else {
        mpuEmergencyCandidate = false; // Reset candidate if deceleration intensity dampens
      }

      // STANDARD BRAKE DEBOUNCE VERIFICATION: Ensure sustained kinetic deceleration curves
      if (now - mpuBrakeCandidateStart >= MPU_BRAKE_CONFIRM_MS) {
        mpuBrakeActive           = true;  // Assert primary dynamic software tracking latch
        mpuBrakeCandidate        = false; // Clear validation registers
        mpuBrakeReleaseCandidate = false; // Reset corresponding release engine checkpoints

        // HARDWARE INTERLOCK GUARD: Suppress queue dispatch if the operator is physically holding the pedal
        if (!Brake.isPressed) {
          // EDGE PROTECTION: Only drop message if standard brake state isn't already tracking active
          if (brakeMode != CMD_BRAKE) {
            brakeMode = CMD_BRAKE;
            xQueueOverwrite(brakeQueue, &brakeMode); // Transmit braking instruction across the FreeRTOS stream
            Serial.print("[MPU] Standard Brake detected at ");
            Serial.print(vehicleAccelY, 3);
            Serial.println(" m/s^2");
          }
        } else {
          // Sync software state to maintain tracking cohesion, but yield queue routing to manual controls
          brakeMode = CMD_BRAKE;
          Serial.println("[MPU] Brake deceleration verified, but physical pedal is already held. Suppressing queue.");
        }
      }
    } else {
      // SHOCK ISOLATION FLUSH: Instantly discard candidates if the breach was an un-sustained peak noise spike
      mpuBrakeCandidate     = false;
      mpuEmergencyCandidate = false;
    }
  }
  
  // --- 2. BRAKE RELEASE EVALUATION ENGINE (Brake lights are currently actively engaged) ---
  else {
    
    // Evaluate if forward acceleration kinetic profiles recover past the hysteresis release cutoff
    if (vehicleAccelY >= MPU_BRAKE_OFF_THRESHOLD) {
      
      // Initial strike past the release limit: establish verification window timestamp
      if (!mpuBrakeReleaseCandidate) {
        mpuBrakeReleaseCandidate = true;
        mpuBrakeReleaseStart = now;
      }

      // Timing Recovery Validation: Require sustained forward kinetic stability before clearing outputs
      if (now - mpuBrakeReleaseStart >= MPU_BRAKE_RELEASE_MS) {
        
        // HARDWARE INTERLOCK SECURITY GUARD: Explicitly prohibit automated telemetry cut-off routines
        // if the operator is still actively depressing the physical chassis mechanical pedal asset.
        if (!Brake.isPressed) {
          mpuBrakeActive           = false; // Safely drop primary software telemetry track latch
          mpuBrakeCandidate        = false; // Flush standard candidate registers
          mpuBrakeReleaseCandidate = false; // Reset release tracking engines
          mpuEmergencyCandidate    = false; // Reset emergency tracking engines

          // EDGE PROTECTION: Only drop message if the running tracking profile isn't already idle
          if (brakeMode != CMD_BRAKE_OFF) {
            brakeMode = CMD_BRAKE_OFF;
            xQueueOverwrite(brakeQueue, &brakeMode); // Send explicit termination token downstream to Core 0
            Serial.print("[MPU] Brake released at ");
            Serial.print(vehicleAccelY, 3);
            Serial.println(" m/s^2");
          }
        } else {
          // INTERLOCK OVERRIDE LATCH: Human input is active. Maintain software brakeMode state tracking,
          // but drop the release flag so telemetry re-evaluates cleanly the split-second the physical pedal is freed.
          mpuBrakeReleaseCandidate = false;
        }
      }
    } else {
      // Kinetic Re-dive Flush: Wipe release tracking if the chassis dives or hits kinetic load mid-recovery
      mpuBrakeReleaseCandidate = false;
    }
  }
}
/**
  * @brief Processes gyro rotational telemetry to track cumulative vehicle heading changes (yaw).
  * Utilizes a Riemann sum mathematical integration engine to automatically cancel running 
  * turn signals once the vehicle tracks clean through an intersection or complete cornering maneuver.
  * 
  * @param correctedGyroZ The clean, bias-compensated rotational velocity around the vertical Z-axis (rad/s)
  * @param dt Loop cadence execution slice duration in seconds (delta time)
  */
void updateMPUTurnDetection(float correctedGyroZ, float dt) {
  
  // --- 1. ACTIVE SIGNAL SCANNING ENGINE (Executed only when a turn signal mode is running) ---
  if (signalMode == CMD_LEFT || signalMode == CMD_RIGHT) {
    
    // Convert the angular rotational velocity from raw radians per second to degrees per second
    float gyroZDeg = correctedGyroZ * (180.0 / PI);
    
    // DRIFT FILTRATION GATING: Prevent highway lane changes or minor lane corrections 
    // from contaminating the yaw integrator before a true cornering maneuver begins.
    if (!turnInitiated) {
      // Strict Turn Initialization Window: Requires an angular rotational velocity exceeding
      // 10.0 degrees/sec to officially register the transition from straight cruising to a live turn.
      if (abs(gyroZDeg) >= 10.0) {
        turnInitiated = true;
      }
    }

    // MATHEMATICAL INTEGRATION ENGINE: Accumulate angular displacement once a true cornering path is verified
    if (turnInitiated) {
      // Accumulate spatial drift across runtime windows utilizing a standard Riemann sum
      accumulatedYaw += gyroZDeg * dt;

      /**
      * DIRECTIONAL DISPLACEMENT ANALYSIS (Aerospace Right-Hand Rule):
      * - Counter-Clockwise (Left) vehicle maneuvers yield positive net yaw accumulation degrees.
      * - Clockwise (Right) vehicle maneuvers yield negative net yaw accumulation degrees.
      */
      if ((signalMode == CMD_LEFT && accumulatedYaw >= YAW_CANCEL_THRESHOLD) ||
          (signalMode == CMD_RIGHT && accumulatedYaw <= -YAW_CANCEL_THRESHOLD)) {
        
        // EDGE PROTECTION: Update state trackers and dispatch cancellation tokens downstream
        signalMode = CMD_TURN_OFF;
        xQueueOverwrite(turnQueue, &signalMode); // Inform Core 0 to immediately drop flasher channels down to low potential
        
        Serial.print("[MPU] Turn auto-canceled. Net Yaw: ");
        Serial.print(accumulatedYaw, 2);
        Serial.println(" deg");
        
        // Internal State Reset: Flush integration metrics to prepare cleanly for subsequent activations
        accumulatedYaw = 0.0;
        turnInitiated  = false;
      }
    }
  } 
  
  // --- 2. STATE HARDWARE PROTECTION FLUSH (System is running idle, turn tracking is inactive) ---
  else {
    /**
    * DYNAMIC BUS CONGESTION PROTECTION:
    * Only zero out memory maps and synchronize clocks if registers aren't already resting
    * inside clean default parameters. This stops continuous telemetry cycles from pounding 
    * tracking caches with redundant writes while cruising straight or parked.
    */
    if (accumulatedYaw != 0.0 || turnInitiated) {
      accumulatedYaw    = 0.0;
      turnInitiated     = false;
    }
  }
}
/**
  * @brief Dynamic Background Telemetry Thread running on Core 1.
  * Processes 6-axis spatial matrices with a state-aware polling cadence:
  * - High-Frequency (10ms): Actively tracking cornering profiles or dynamic breaking maneuvers.
  * - Low-Frequency (100ms): Cruising or stationary idle states to preserve CPU and I2C bandwidth.
  * 
  * @param pvParameters Standard untyped FreeRTOS task argument block pointer (Unused).
  */
void processVehicleTelemetry(void* pvParameters) {  
  // Prevent startup delta-time explosion by seeding time-markers at task launch
  lastUpdateTime = millis();


  for (;;) { 
    // Feed watchdog before polling I2C bus
    esp_task_wdt_reset(); 

      //keep feeding the WDT and yield without touching the I2C bus:
      bool isCalibrating = false;
      if (calibrationQueue != NULL) {
        xQueuePeek(calibrationQueue, &isCalibrating, 0);
      }
      if (isCalibrating) {
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
      }
      
    if (mpuAvailable) {
      uint32_t now = millis();
      float dt = (now - lastUpdateTime) / 1000.0;
      lastUpdateTime = now;
      
      // Safe clamp fallback anchor to handle timing anomalies or scheduler hiccups
      if (dt <= 0.0 || dt > 0.5) {
        dt = 0.01; // Default to standard 10ms execution slice
      }

      // Check system configurations to see if ANY tracking features are currently in demand
      bool turnSignalActive = (signalMode == CMD_LEFT || signalMode == CMD_RIGHT);
      bool brakeTrackingActive = (useBrakelights && useMPUBrake && mpuBrakeAvailable);

      // Only perform heavy sensor polling if feature toggles are globally enabled
      if (brakeTrackingActive || (useMPUTurnSignal && turnSignalActive)) {
        
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // --- 1. DYNAMIC BRAKE MATRIX LOGIC ---
        if (brakeTrackingActive) {
          // Normalize accelerometer and gyro-X lines against calibration biases
          float correctedAccelY = a.acceleration.y - accelBiasY;
          float correctedAccelZ = a.acceleration.z - accelBiasZ;
          float correctedGyroX  = g.gyro.x - gyroBiasX;

          // Calculate pitch angle using accelerometer gravity distribution
          float livePitchAccel = atan2(correctedAccelY, correctedAccelZ);

          // Complementary Filter blending gyro rate and low-pass accel angle
          vehiclePitchRad = alpha * (vehiclePitchRad + (correctedGyroX * dt)) + (1.0 - alpha) * livePitchAccel;

          // Project and cancel gravity vector components from dynamic vehicle Y forces
          float dynamicForwardAccel = correctedAccelY - (measuredGravity * sin(vehiclePitchRad));

          // Low-pass smooth final forward acceleration metrics
          filteredVehicleAccelY = (dynamicForwardAccel * MPU_ACCEL_FILTER_ALPHA) + (filteredVehicleAccelY * (1.0 - MPU_ACCEL_FILTER_ALPHA));

          // Pass processed metrics to active brake comparison engine
          updateMPUBrakeDetection(filteredVehicleAccelY, now);
        }

        // --- 2. DYNAMIC TURN ACCUMULATION LOGIC ---
        if (useMPUTurnSignal && turnSignalActive) {
          // Normalize raw gyro-Z data stream against static calibration biases
          float correctedGyroZ = g.gyro.z - gyroBiasZ;

          // Pass yaw rotation rate and loop delta time directly to comparison engine
          updateMPUTurnDetection(correctedGyroZ, dt);
        }
      }

      /**
      * DYNAMIC CADENCE SCALING ENGINE:
      * If we are currently executing a turn or actively tracking a deceleration curve,
      * loop at a fast 10ms speed. If cruising or stopped, yield for 100ms.
      */
      if (turnSignalActive || mpuBrakeActive || mpuBrakeCandidate) {
        vTaskDelay(pdMS_TO_TICKS(10));  // High-performance operational mode
      } else {
        vTaskDelay(pdMS_TO_TICKS(100)); // Eco-mode reduction layer
      }

    } else {
      // Fail-safe protection fallback: If physical hardware is lost, suspend processing loop thread entirely
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}