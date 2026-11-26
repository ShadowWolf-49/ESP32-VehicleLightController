#ifndef WEB_PORTAL_H
#define WEB_PORTAL_H

#include "config.h"

String buildDiagnosticsHTML();
String buildConfigHTML(String statusBanner = "");
String buildAboutHTML();

void handleDiagnosticsJSON();
void handleDiagnosticsPage();
void handleRootPage();
void handleSavePage();
void handleCalibratePage();
void handleResetDefaultsPage();
void handleRebootPage();
void handleAboutPage();
void handleOtaComplete();
void handleOtaUpload();

void webConfigPortalTask(void* pvParameters);
void startWebPortal();
void stopWebPortal();

#endif // WEB_PORTAL_H