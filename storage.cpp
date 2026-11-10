#include "storage.h"

/**
   * @brief Commits all current runtime configuration settings to ESP32 Flash (NVS).
   */
void saveSystemConfig() {
  prefs.begin("SysConfig", false); // Open in Read/Write mode

  // Normal Booleans & Timing
  prefs.putUInt("debounce",     DEBOUNCE_LOCKOUT);
  prefs.putBool("ignInterlock", useIgnitionInterlock);
  prefs.putBool("diagOn",       diagnosticsOn);
  prefs.putBool("useBrake",     useBrakelights);
  prefs.putBool("useMpuBrake",  useMPUBrake);
  prefs.putBool("useMpuTurn",   useMPUTurnSignal);

  // Brake-light variables
  prefs.putUInt("brkFlashInt",  brakeFlashInterval);
  prefs.putUInt("brkFlashDur",  brakeFlashDuration);
  prefs.putUInt("emgFlashInt",  EmergencyFlashInterval);
  prefs.putUInt("emgFlashDur",  EmergencyFlashDuration);

  // Turn-signal variables
  prefs.putUInt("blkFlashInt",  blinkFlashInterval);
  prefs.putUInt("holdDurTurn",  HoldDurationTurning);
  prefs.putUInt("holdDurInf",   HoldDurationInfinite);
  prefs.putUInt("durLaneChg",   DurationLaneChange);
  prefs.putUInt("flashDurTurn", FlashDurationTurning);

  // Hazard variables
  prefs.putUInt("hazFlashInt",  hazardFlashInterval);
  prefs.putFloat("brkHazPct",   brakeHazardPercent);

  // MPU6050 calibration variables
  prefs.putInt("samples",       samples);
  prefs.putFloat("minMountX",   MIN_MOUNT_ANGLE_X);
  prefs.putFloat("maxMountX",   MAX_MOUNT_ANGLE_X);
  prefs.putFloat("vehYDir",     VEHICLE_Y_DIRECTION);

  // MPU6050 filtering
  prefs.putFloat("accelAlpha",  MPU_ACCEL_FILTER_ALPHA);
  prefs.putFloat("compAlpha",   alpha);

  // MPU6050 normal brake detection
  prefs.putFloat("brkOnThresh", MPU_BRAKE_ON_THRESHOLD);
  prefs.putFloat("brkOffThres", MPU_BRAKE_OFF_THRESHOLD);
  prefs.putUInt("brkConfMs",    MPU_BRAKE_CONFIRM_MS);
  prefs.putUInt("brkRelMs",     MPU_BRAKE_RELEASE_MS);

  // MPU6050 emergency brake detection
  prefs.putFloat("emgThresh",   MPU_EMERGENCY_THRESHOLD);
  prefs.putUInt("emgConfMs",    MPU_EMERGENCY_CONFIRM_MS);

  // MPU6050 turn-signal
  prefs.putFloat("yawCancel",   YAW_CANCEL_THRESHOLD);

  // Wi-Fi Portal settings
  prefs.putString("apSsid",     AP_SSID);
  prefs.putString("apPass",     AP_PASS);
  prefs.putUInt("portalTime",   PORTAL_TIMEOUT_MS);

  prefs.putBool("cfgInit",      true); // Mark flash configuration as initialized
  prefs.end();

  Serial.println("[CONFIG] System configuration parameters saved to NVS flash.");
}
/**
  * @brief Loads configuration settings from Flash (NVS). 
  * If flash is empty (first boot), writes the default variables to Flash.
  */
void loadSystemConfig() {
  prefs.begin("SysConfig", false);

  if (!prefs.getBool("cfgInit", false)) {
    prefs.end();
    Serial.println("[CONFIG] No saved configuration found in Flash. Writing defaults...");
    saveSystemConfig();
    return;
  }

  // Normal Booleans & Timing
  DEBOUNCE_LOCKOUT        = prefs.getUInt("debounce",     DEBOUNCE_LOCKOUT);
  useIgnitionInterlock    = prefs.getBool("ignInterlock", useIgnitionInterlock);
  diagnosticsOn           = prefs.getBool("diagOn",       diagnosticsOn);
  useBrakelights          = prefs.getBool("useBrake",     useBrakelights);
  useMPUBrake             = prefs.getBool("useMpuBrake",  useMPUBrake);
  useMPUTurnSignal        = prefs.getBool("useMpuTurn",   useMPUTurnSignal);

  // Brake-light variables
  brakeFlashInterval      = prefs.getUInt("brkFlashInt",  brakeFlashInterval);
  brakeFlashDuration      = prefs.getUInt("brkFlashDur",  brakeFlashDuration);
  EmergencyFlashInterval  = prefs.getUInt("emgFlashInt",  EmergencyFlashInterval);
  EmergencyFlashDuration  = prefs.getUInt("emgFlashDur",  EmergencyFlashDuration);

  // Turn-signal variables
  blinkFlashInterval      = prefs.getUInt("blkFlashInt",  blinkFlashInterval);
  HoldDurationTurning     = prefs.getUInt("holdDurTurn",  HoldDurationTurning);
  HoldDurationInfinite    = prefs.getUInt("holdDurInf",   HoldDurationInfinite);
  DurationLaneChange      = prefs.getUInt("durLaneChg",   DurationLaneChange);
  FlashDurationTurning    = prefs.getUInt("flashDurTurn", FlashDurationTurning);

  // Hazard variables
  hazardFlashInterval     = prefs.getUInt("hazFlashInt",  hazardFlashInterval);
  brakeHazardPercent      = prefs.getFloat("brkHazPct",   brakeHazardPercent);

  // MPU6050 calibration variables
  samples                 = prefs.getInt("samples",       samples);
  MIN_MOUNT_ANGLE_X       = prefs.getFloat("minMountX",   MIN_MOUNT_ANGLE_X);
  MAX_MOUNT_ANGLE_X       = prefs.getFloat("maxMountX",   MAX_MOUNT_ANGLE_X);
  VEHICLE_Y_DIRECTION     = prefs.getFloat("vehYDir",     VEHICLE_Y_DIRECTION);

  // MPU6050 filtering
  MPU_ACCEL_FILTER_ALPHA  = prefs.getFloat("accelAlpha",  MPU_ACCEL_FILTER_ALPHA);
  alpha                   = prefs.getFloat("compAlpha",   alpha);

  // MPU6050 normal brake detection
  MPU_BRAKE_ON_THRESHOLD  = prefs.getFloat("brkOnThresh", MPU_BRAKE_ON_THRESHOLD);
  MPU_BRAKE_OFF_THRESHOLD = prefs.getFloat("brkOffThres", MPU_BRAKE_OFF_THRESHOLD);
  MPU_BRAKE_CONFIRM_MS    = prefs.getUInt("brkConfMs",    MPU_BRAKE_CONFIRM_MS);
  MPU_BRAKE_RELEASE_MS    = prefs.getUInt("brkRelMs",     MPU_BRAKE_RELEASE_MS);

  // MPU6050 emergency brake detection
  MPU_EMERGENCY_THRESHOLD = prefs.getFloat("emgThresh",   MPU_EMERGENCY_THRESHOLD);
  MPU_EMERGENCY_CONFIRM_MS= prefs.getUInt("emgConfMs",    MPU_EMERGENCY_CONFIRM_MS);

  // MPU6050 turn-signal
  YAW_CANCEL_THRESHOLD    = prefs.getFloat("yawCancel",   YAW_CANCEL_THRESHOLD);

  // Wi-Fi Portal settings
  AP_SSID                 = prefs.getString("apSsid",   AP_SSID);
  AP_PASS                 = prefs.getString("apPass",   AP_PASS);
  PORTAL_TIMEOUT_MS       = prefs.getUInt("portalTime", PORTAL_TIMEOUT_MS);

  prefs.end();
  Serial.println("[CONFIG] Loaded configuration settings from NVS flash.");
}
/**
  * @brief Restores all configuration settings back to factory defaults in RAM and NVS Flash.
  */
void resetSystemConfigDefaults() {
  DEBOUNCE_LOCKOUT         = 15;
  useIgnitionInterlock     = false;
  diagnosticsOn            = false;
  useBrakelights           = true;
  useMPUBrake              = true;
  useMPUTurnSignal         = true;

  brakeFlashInterval       = 140;
  brakeFlashDuration       = 1000;
  EmergencyFlashInterval   = 70;
  EmergencyFlashDuration   = 1000;

  blinkFlashInterval       = 400;
  HoldDurationTurning      = 1500;
  HoldDurationInfinite     = 5000;
  DurationLaneChange       = 5000;
  FlashDurationTurning     = 20000;

  hazardFlashInterval      = 450;
  brakeHazardPercent       = 70.0;

  samples                  = 300;
  MIN_MOUNT_ANGLE_X        = -45.0;
  MAX_MOUNT_ANGLE_X        = 45.0;
  VEHICLE_Y_DIRECTION      = 1.0;

  MPU_ACCEL_FILTER_ALPHA   = 0.15;
  alpha                    = 0.98;

  MPU_BRAKE_ON_THRESHOLD   = -1.75;
  MPU_BRAKE_OFF_THRESHOLD  = -0.80;
  MPU_BRAKE_CONFIRM_MS     = 120;
  MPU_BRAKE_RELEASE_MS     = 250;

  MPU_EMERGENCY_THRESHOLD  = -5.0;
  MPU_EMERGENCY_CONFIRM_MS = 80;

  YAW_CANCEL_THRESHOLD     = 15.0;

  AP_SSID                  = "Vehicle_Light_Config";
  AP_PASS                  = "12345678";
  PORTAL_TIMEOUT_MS        = 300000;

  saveSystemConfig();
  Serial.println("[CONFIG] All settings restored to factory defaults and saved to Flash.");
}