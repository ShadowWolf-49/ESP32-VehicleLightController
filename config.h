#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Preferences.h>
#include <WebServer.h>

// ===================================================================================
// =======================        Hardware Constants         =========================
// ===================================================================================
const uint32_t WDT_TIMEOUT_SEC = 3;
const uint8_t NUM_BUTTONS      = 8;
const uint8_t NUM_RELAYS       = 8;
const uint8_t ledPin           = 23;

// ===================================================================================
// =======================       Core State Definitions      =========================
// ===================================================================================
enum SignalCommand {
  CMD_BRAKE,
  CMD_EMERGENCY_BRAKE,
  CMD_BRAKE_OFF,
  CMD_LEFT,
  CMD_RIGHT,
  CMD_TURN_OFF,
  CMD_HAZARD,
  CMD_HAZARD_OFF,
  CMD_IGNITION_ON,
  CMD_IGNITION_OFF
};

enum OtaEvent : uint8_t {
  OTA_EVENT_NONE = 0,
  OTA_EVENT_START,
  OTA_EVENT_ABORT
};

struct Button {
  const uint8_t value;
  volatile bool isPressed = false;
  volatile bool eventTriggered = false;
  uint32_t lastEdgeTime = 0;          // Exclusive to buttonISR (debounce)
  uint32_t pressStartTime = 0;        // Exclusive to Core 1 (hold duration)

  operator uint8_t() const { return value; }
  operator bool() const volatile { return isPressed; }
  Button& operator=(const Button&) = delete;

  bool operator=(bool state) volatile {
    if (isPressed != state) {
      isPressed = state;
    }
    return isPressed;
  }
};

struct Relay {
  const uint8_t value;
  volatile bool isOn = false;

  operator uint8_t() const { return value; }
  operator bool() const volatile { return isOn; }
  Relay& operator=(const Relay&) = delete;

  bool operator=(bool state) volatile {
    if (isOn != state) {
      isOn = state;
      digitalWrite(value, isOn ? HIGH : LOW);
    }
    return isOn;
  }
};

// ===================================================================================
// =======================    User Configuration Settings    =========================
// ===================================================================================
extern uint32_t DEBOUNCE_LOCKOUT;
extern bool useIgnitionInterlock;
extern bool diagnosticsOn;
extern bool useBrakelights;
extern bool useMPUBrake;
extern bool useMPUTurnSignal;

// Brake-light variables
extern uint32_t brakeFlashInterval;
extern uint32_t brakeFlashDuration;
extern uint32_t EmergencyFlashInterval;
extern uint32_t EmergencyFlashDuration;

// Turn-signal variables
extern uint32_t blinkFlashInterval;
extern uint32_t HoldDurationTurning;
extern uint32_t HoldDurationInfinite;
extern uint32_t DurationLaneChange;
extern uint32_t FlashDurationTurning;

// Hazard variables
extern uint32_t hazardFlashInterval;
extern float brakeHazardPercent;

// MPU6050 calibration variables
extern int samples;
extern float MIN_MOUNT_ANGLE_X;
extern float MAX_MOUNT_ANGLE_X;
extern float VEHICLE_Y_DIRECTION;

// MPU6050 filtering & detection thresholds
extern float MPU_ACCEL_FILTER_ALPHA;
extern float alpha;
extern float MPU_BRAKE_ON_THRESHOLD;
extern float MPU_BRAKE_OFF_THRESHOLD;
extern uint32_t MPU_BRAKE_CONFIRM_MS;
extern uint32_t MPU_BRAKE_RELEASE_MS;
extern float MPU_EMERGENCY_THRESHOLD;
extern uint32_t MPU_EMERGENCY_CONFIRM_MS;
extern float YAW_CANCEL_THRESHOLD;

// Wi-Fi Configuration Portal Settings
extern String AP_SSID;
extern String AP_PASS;
extern uint32_t PORTAL_TIMEOUT_MS;

// ===================================================================================
// =======================     Runtime & System Globals      =========================
// ===================================================================================
// Webserver & Storage
extern WebServer configServer;
extern uint32_t portalStartTime;

// Brake & Ignition Trackers
extern uint32_t brakeStartTime;
extern bool lastBrakeState;
extern volatile bool mpuBrakeActive;
extern volatile uint32_t activeTimeout;

// MPU6050 Objects & State Flags
extern Adafruit_MPU6050 mpu;
extern bool mpuAvailable;
extern bool mpuBrakeAvailable;

extern bool mpuBrakeCandidate;
extern bool mpuBrakeReleaseCandidate;
extern bool mpuEmergencyCandidate;
extern uint32_t mpuBrakeCandidateStart;
extern uint32_t mpuBrakeReleaseStart;
extern uint32_t mpuEmergencyCandidateStart;

extern float accumulatedYaw;
extern bool turnInitiated;

// MPU6050 Biases & Orientation
extern float accelBiasX;
extern float accelBiasY;
extern float accelBiasZ;
extern float gyroBiasX;
extern float gyroBiasY;
extern float gyroBiasZ;
extern float measuredGravity;
extern float staticMountPitchRad;
extern float staticMountPitchDeg;
extern float vehiclePitchRad;
extern uint32_t lastUpdateTime;
extern float filteredVehicleAccelY;

// FreeRTOS Queues
extern QueueHandle_t brakeQueue;
extern QueueHandle_t turnQueue;
extern QueueHandle_t hazardQueue;
extern QueueHandle_t ignitionQueue;
extern QueueHandle_t hornQueue;
extern QueueHandle_t turnCancelQueue;
extern QueueHandle_t calibrationQueue;
extern QueueHandle_t otaQueue;

extern QueueHandle_t turnToBrakeQueue;
extern QueueHandle_t brakeToTurnQueue;
extern QueueHandle_t brakeToHazardQueue;
extern QueueHandle_t hazardToBrakeQueue;
extern QueueHandle_t turnToHazardQueue;
extern QueueHandle_t hazardToTurnQueue;

// FreeRTOS Task Handles
extern TaskHandle_t telemetryHandle;
extern TaskHandle_t brakeTaskHandle;
extern TaskHandle_t turnTaskHandle;
extern TaskHandle_t inputTaskHandle;
extern TaskHandle_t hazardTaskHandle;
extern TaskHandle_t hornTaskHandle;
extern TaskHandle_t webPortalTaskHandle;
extern TaskHandle_t diagnosticsTaskHandle;
extern TaskHandle_t core0WdtHandle;

// Global Mode Trackers
extern SignalCommand signalMode;
extern SignalCommand brakeMode;
extern SignalCommand hazardMode;
extern SignalCommand ignitionMode;

// Hardware Button & Relay Objects
extern Button Left;
extern Button Right;
extern Button Hazard;
extern Button Horn;
extern Button Brake;
extern Button Ignition;
extern Button SpareButton1;
extern Button SpareButton2;

extern Relay FrontLeft;
extern Relay FrontRight;
extern Relay BackLeft;
extern Relay BackRight;
extern Relay HornRelay;
extern Relay SpareRelay1;
extern Relay SpareRelay2;
extern Relay SpareRelay3;

extern Button* buttons[NUM_BUTTONS];
extern Relay* relays[NUM_RELAYS];

#endif // CONFIG_H