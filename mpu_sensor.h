#ifndef MPU_SENSOR_H
#define MPU_SENSOR_H

#include "config.h"

float vectorMagnitude(float x, float y, float z);
bool performSensorCalibration();
void initializeMPUCalibration();
void updateMPUBrakeDetection(float vehicleAccelY, uint32_t now);
void updateMPUTurnDetection(float correctedGyroZ, float dt);
void processVehicleTelemetry(void* pvParameters);

#endif // MPU_SENSOR_H