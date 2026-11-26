#include "storage.h"

/**
   * @brief Commits all current runtime configuration settings to ESP32 Flash (NVS).
   */
void saveSystemConfig() {
  Preferences sysPrefs; // Local instance prevents NVS collisions
  sysPrefs.begin("SysConfig", false); // Open in Read only mode

  // Normal Booleans & Timing
  sysPrefs.putUInt("debounce",     DEBOUNCE_LOCKOUT);
  sysPrefs.putBool("ignInterlock", useIgnitionInterlock);
  sysPrefs.putBool("diagOn",       diagnosticsOn);
  sysPrefs.putBool("useBrake",     useBrakelights);
  sysPrefs.putBool("useMpuBrake",  useMPUBrake);
  sysPrefs.putBool("useMpuTurn",   useMPUTurnSignal);

  // Brake-light variables
  sysPrefs.putUInt("brkFlashInt",  brakeFlashInterval);
  sysPrefs.putUInt("brkFlashDur",  brakeFlashDuration);
  sysPrefs.putUInt("emgFlashInt",  EmergencyFlashInterval);
  sysPrefs.putUInt("emgFlashDur",  EmergencyFlashDuration);

  // Turn-signal variables
  sysPrefs.putUInt("blkFlashInt",  blinkFlashInterval);
  sysPrefs.putUInt("holdDurTurn",  HoldDurationTurning);
  sysPrefs.putUInt("holdDurInf",   HoldDurationInfinite);
  sysPrefs.putUInt("durLaneChg",   DurationLaneChange);
  sysPrefs.putUInt("flashDurTurn", FlashDurationTurning);

  // Hazard variables
  sysPrefs.putUInt("hazFlashInt",  hazardFlashInterval);
  sysPrefs.putFloat("brkHazPct",   brakeHazardPercent);

  // MPU6050 calibration variables
  sysPrefs.putInt("samples",       samples);
  sysPrefs.putFloat("minMountX",   MIN_MOUNT_ANGLE_X);
  sysPrefs.putFloat("maxMountX",   MAX_MOUNT_ANGLE_X);
  sysPrefs.putFloat("vehYDir",     VEHICLE_Y_DIRECTION);

  // MPU6050 filtering
  sysPrefs.putFloat("accelAlpha",  MPU_ACCEL_FILTER_ALPHA);
  sysPrefs.putFloat("compAlpha",   alpha);

  // MPU6050 normal brake detection
  sysPrefs.putFloat("brkOnThresh", MPU_BRAKE_ON_THRESHOLD);
  sysPrefs.putFloat("brkOffThres", MPU_BRAKE_OFF_THRESHOLD);
  sysPrefs.putUInt("brkConfMs",    MPU_BRAKE_CONFIRM_MS);
  sysPrefs.putUInt("brkRelMs",     MPU_BRAKE_RELEASE_MS);

  // MPU6050 emergency brake detection
  sysPrefs.putFloat("emgThresh",   MPU_EMERGENCY_THRESHOLD);
  sysPrefs.putUInt("emgConfMs",    MPU_EMERGENCY_CONFIRM_MS);

  // MPU6050 turn-signal
  sysPrefs.putFloat("yawCancel",   YAW_CANCEL_THRESHOLD);

  // Wi-Fi Portal settings
  sysPrefs.putString("apSsid",     AP_SSID);
  sysPrefs.putString("apPass",     AP_PASS);
  sysPrefs.putUInt("portalTime",   PORTAL_TIMEOUT_MS);

  sysPrefs.putBool("cfgInit",      true); // Mark flash configuration as initialized
  sysPrefs.end();

  Serial.println("[CONFIG] System configuration parameters saved to NVS flash.");
}
/**
  * @brief Loads configuration settings from Flash (NVS). 
  * If flash is empty (first boot), writes the default variables to Flash.
  */
void loadSystemConfig() {
  Preferences sysPrefs; // Local instance prevents NVS collisions
  sysPrefs.begin("SysConfig", false);

  if (!sysPrefs.getBool("cfgInit", false)) {
    sysPrefs.end();
    Serial.println("[CONFIG] No saved configuration found in Flash. Writing defaults...");
    saveSystemConfig();
    return;
  }

  // Normal Booleans & Timing
  DEBOUNCE_LOCKOUT        = sysPrefs.getUInt("debounce",     DEBOUNCE_LOCKOUT);
  useIgnitionInterlock    = sysPrefs.getBool("ignInterlock", useIgnitionInterlock);
  diagnosticsOn           = sysPrefs.getBool("diagOn",       diagnosticsOn);
  useBrakelights          = sysPrefs.getBool("useBrake",     useBrakelights);
  useMPUBrake             = sysPrefs.getBool("useMpuBrake",  useMPUBrake);
  useMPUTurnSignal        = sysPrefs.getBool("useMpuTurn",   useMPUTurnSignal);

  // Brake-light variables
  brakeFlashInterval      = sysPrefs.getUInt("brkFlashInt",  brakeFlashInterval);
  brakeFlashDuration      = sysPrefs.getUInt("brkFlashDur",  brakeFlashDuration);
  EmergencyFlashInterval  = sysPrefs.getUInt("emgFlashInt",  EmergencyFlashInterval);
  EmergencyFlashDuration  = sysPrefs.getUInt("emgFlashDur",  EmergencyFlashDuration);

  // Turn-signal variables
  blinkFlashInterval      = sysPrefs.getUInt("blkFlashInt",  blinkFlashInterval);
  HoldDurationTurning     = sysPrefs.getUInt("holdDurTurn",  HoldDurationTurning);
  HoldDurationInfinite    = sysPrefs.getUInt("holdDurInf",   HoldDurationInfinite);
  DurationLaneChange      = sysPrefs.getUInt("durLaneChg",   DurationLaneChange);
  FlashDurationTurning    = sysPrefs.getUInt("flashDurTurn", FlashDurationTurning);

  // Hazard variables
  hazardFlashInterval     = sysPrefs.getUInt("hazFlashInt",  hazardFlashInterval);
  brakeHazardPercent      = sysPrefs.getFloat("brkHazPct",   brakeHazardPercent);

  // MPU6050 calibration variables
  samples                 = sysPrefs.getInt("samples",       samples);
  MIN_MOUNT_ANGLE_X       = sysPrefs.getFloat("minMountX",   MIN_MOUNT_ANGLE_X);
  MAX_MOUNT_ANGLE_X       = sysPrefs.getFloat("maxMountX",   MAX_MOUNT_ANGLE_X);
  VEHICLE_Y_DIRECTION     = sysPrefs.getFloat("vehYDir",     VEHICLE_Y_DIRECTION);

  // MPU6050 filtering
  MPU_ACCEL_FILTER_ALPHA  = sysPrefs.getFloat("accelAlpha",  MPU_ACCEL_FILTER_ALPHA);
  alpha                   = sysPrefs.getFloat("compAlpha",   alpha);

  // MPU6050 normal brake detection
  MPU_BRAKE_ON_THRESHOLD  = sysPrefs.getFloat("brkOnThresh", MPU_BRAKE_ON_THRESHOLD);
  MPU_BRAKE_OFF_THRESHOLD = sysPrefs.getFloat("brkOffThres", MPU_BRAKE_OFF_THRESHOLD);
  MPU_BRAKE_CONFIRM_MS    = sysPrefs.getUInt("brkConfMs",    MPU_BRAKE_CONFIRM_MS);
  MPU_BRAKE_RELEASE_MS    = sysPrefs.getUInt("brkRelMs",     MPU_BRAKE_RELEASE_MS);

  // MPU6050 emergency brake detection
  MPU_EMERGENCY_THRESHOLD = sysPrefs.getFloat("emgThresh",   MPU_EMERGENCY_THRESHOLD);
  MPU_EMERGENCY_CONFIRM_MS= sysPrefs.getUInt("emgConfMs",    MPU_EMERGENCY_CONFIRM_MS);

  // MPU6050 turn-signal
  YAW_CANCEL_THRESHOLD    = sysPrefs.getFloat("yawCancel",   YAW_CANCEL_THRESHOLD);

  // Wi-Fi Portal settings
  AP_SSID                 = sysPrefs.getString("apSsid",   AP_SSID);
  AP_PASS                 = sysPrefs.getString("apPass",   AP_PASS);
  PORTAL_TIMEOUT_MS       = sysPrefs.getUInt("portalTime", PORTAL_TIMEOUT_MS);

  sysPrefs.end();
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