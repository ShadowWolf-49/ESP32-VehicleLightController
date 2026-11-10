#ifndef RELAYS_H
#define RELAYS_H

#include "config.h"

void groundAllRelaysFailSafe();
void brakeLightTask(void* pvParameters);
void turnSignalTask(void* pvParameters);
void hazardTask(void* pvParameters);
void hornTask(void* pvParameters);

#endif // RELAYS_H