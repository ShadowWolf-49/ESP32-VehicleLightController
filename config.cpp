#include "config.h"

// ===================================================================================
// =======================    User Configuration Settings    =========================
// ===================================================================================
uint32_t DEBOUNCE_LOCKOUT        = 15;
bool useIgnitionInterlock        = false;
bool diagnosticsOn               = false;
bool useBrakelights              = true;
bool useMPUBrake                 = true;
bool useMPUTurnSignal            = true;

// Brake-light variables
uint32_t brakeFlashInterval      = 140;
uint32_t brakeFlashDuration      = 1000;
uint32_t EmergencyFlashInterval  = 70;
uint32_t EmergencyFlashDuration  = 1000;

// Turn-signal variables
uint32_t blinkFlashInterval      = 400;
uint32_t HoldDurationTurning     = 1500;
uint32_t HoldDurationInfinite    = 5000;
uint32_t DurationLaneChange      = 5000;
uint32_t FlashDurationTurning    = 20000;

// Hazard variables
uint32_t hazardFlashInterval     = 450;
float brakeHazardPercent         = 70.0;

// MPU6050 calibration variables
int samples                      = 300;
float MIN_MOUNT_ANGLE_X          = -45.0;
float MAX_MOUNT_ANGLE_X          = 45.0;
float VEHICLE_Y_DIRECTION        = 1.0;

// MPU6050 filtering & detection thresholds
float MPU_ACCEL_FILTER_ALPHA     = 0.15;
float alpha                      = 0.98;
float MPU_BRAKE_ON_THRESHOLD     = -1.75;
float MPU_BRAKE_OFF_THRESHOLD    = -0.80;
uint32_t MPU_BRAKE_CONFIRM_MS    = 120;
uint32_t MPU_BRAKE_RELEASE_MS    = 250;
float MPU_EMERGENCY_THRESHOLD    = -5.0;
uint32_t MPU_EMERGENCY_CONFIRM_MS = 80;
float YAW_CANCEL_THRESHOLD       = 15.0;

// Wi-Fi Configuration Portal Settings
String AP_SSID                   = "Vehicle_Light_Config";
String AP_PASS                   = "12345678";
uint32_t PORTAL_TIMEOUT_MS       = 300000;

// ===================================================================================
// =======================     Runtime & System Globals      =========================
// ===================================================================================
WebServer configServer(80);
uint32_t portalStartTime         = 0;
Preferences prefs;

uint32_t brakeStartTime          = 0;
bool lastBrakeState              = false;
volatile bool mpuBrakeActive     = false;
volatile uint32_t activeTimeout  = 5000;

Adafruit_MPU6050 mpu;
bool mpuAvailable                = false;
bool mpuBrakeAvailable           = false;

bool mpuBrakeCandidate           = false;
bool mpuBrakeReleaseCandidate    = false;
bool mpuEmergencyCandidate       = false;
uint32_t mpuBrakeCandidateStart  = 0;
uint32_t mpuBrakeReleaseStart    = 0;
uint32_t mpuEmergencyCandidateStart = 0;

float accumulatedYaw             = 0.0;
bool turnInitiated               = false;

float accelBiasX                 = 0.0;
float accelBiasY                 = 0.0;
float accelBiasZ                 = 0.0;
float gyroBiasX                  = 0.0;
float gyroBiasY                  = 0.0;
float gyroBiasZ                  = 0.0;
float measuredGravity            = 9.81;
float staticMountPitchRad        = 0.0;
float staticMountPitchDeg        = 0.0;
float vehiclePitchRad            = 0.0;
uint32_t lastUpdateTime          = 0;
float filteredVehicleAccelY      = 0.0;

// FreeRTOS Queues
QueueHandle_t brakeQueue         = NULL;
QueueHandle_t turnQueue          = NULL;
QueueHandle_t hazardQueue        = NULL;
QueueHandle_t ignitionQueue      = NULL;
QueueHandle_t hornQueue          = NULL;
QueueHandle_t turnCancelQueue    = NULL;

QueueHandle_t turnToBrakeQueue   = NULL;
QueueHandle_t brakeToTurnQueue   = NULL;
QueueHandle_t brakeToHazardQueue = NULL;
QueueHandle_t hazardToBrakeQueue = NULL;
QueueHandle_t turnToHazardQueue  = NULL;
QueueHandle_t hazardToTurnQueue  = NULL;

// FreeRTOS Task Handles
TaskHandle_t telemetryHandle     = NULL;
TaskHandle_t brakeTaskHandle     = NULL;
TaskHandle_t turnTaskHandle      = NULL;
TaskHandle_t inputTaskHandle     = NULL;
TaskHandle_t hazardTaskHandle    = NULL;
TaskHandle_t hornTaskHandle      = NULL;
TaskHandle_t webPortalTaskHandle = NULL;
TaskHandle_t core0WdtHandle      = NULL;

// Global Mode Trackers
SignalCommand signalMode         = CMD_TURN_OFF;
SignalCommand brakeMode          = CMD_BRAKE_OFF;
SignalCommand hazardMode         = CMD_HAZARD_OFF;
SignalCommand ignitionMode       = CMD_IGNITION_OFF;

// Hardware Button Objects
Button Left                      = {.value = 18};
Button Right                     = {.value = 19};
Button Hazard                    = {.value = 5};
Button Horn                      = {.value = 16};
Button Brake                     = {.value = 17};
Button Ignition                  = {.value = 4};
Button SpareButton1              = {.value = 2};
Button SpareButton2              = {.value = 15};

// Hardware Relay Objects
Relay FrontLeft                  = {.value = 32};
Relay FrontRight                 = {.value = 33};
Relay BackLeft                   = {.value = 25};
Relay BackRight                  = {.value = 26};
Relay HornRelay                  = {.value = 27};
Relay SpareRelay1                = {.value = 14};
Relay SpareRelay2                = {.value = 12};
Relay SpareRelay3                = {.value = 13};

Button* buttons[NUM_BUTTONS]     = {&Left, &Right, &Hazard, &Horn, &Brake, &Ignition, &SpareButton1, &SpareButton2};
Relay* relays[NUM_RELAYS]        = {&FrontLeft, &FrontRight, &BackLeft, &BackRight, &HornRelay, &SpareRelay1, &SpareRelay2, &SpareRelay3};