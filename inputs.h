#ifndef INPUTS_H
#define INPUTS_H

#include "config.h"

void buttonISR(void* arg);
void detectBrakePedal();
void processTurnSignalButtons();
void processHazardButton();
void processHornButton();
bool processIgnitionButton();
void processPortalComboButton();
void mainInputProcessorTask(void* pvParameters);

#endif // INPUTS_H