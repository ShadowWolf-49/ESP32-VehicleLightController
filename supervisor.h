#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include "config.h"

void processSystemInterlockSupervisor(void* pvParameters);
void systemDiagnosticsTask(void* pvParameters);
void core0WatchdogTask(void* pvParameters);

#endif // SUPERVISOR_H