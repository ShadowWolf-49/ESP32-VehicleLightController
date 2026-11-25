#include "web_portal.h"
#include "storage.h"
#include "mpu_sensor.h"
#include <WiFi.h>
#include <esp_task_wdt.h>

/**
  * @brief Builds the live AJAX-enabled diagnostics webpage with real-time relay badges.
  */
String buildDiagnosticsHTML() {
  String html = F("<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
                  "<title>Live System Diagnostics</title><style>"
                  "body{font-family:Arial,sans-serif;background:#121212;color:#eee;margin:0;padding:15px;}"
                  "h2{color:#4db8ff;margin-bottom:5px;}h3{color:#ffcc00;border-bottom:1px solid #333;padding-bottom:4px;margin-top:20px;}"
                  ".timer{color:#aaa;font-size:0.9em;margin-bottom:12px;}"
                  ".nav{display:flex;gap:10px;margin-bottom:15px;align-items:center;}"
                  ".btn{padding:8px 14px;background:#0288d1;color:#fff;text-decoration:none;border:none;border-radius:5px;font-weight:bold;font-size:0.9em;cursor:pointer;}"
                  ".btn-back{background:#37474f;}.btn-pause{background:#ef6c00;}"
                  ".card{background:#1e1e1e;padding:10px;border-radius:5px;margin-bottom:10px;}"
                  ".row{display:flex;justify-content:space-between;padding:5px 0;border-bottom:1px solid #2a2a2a;font-size:0.92em;}"
                  ".row:last-child{border-bottom:none;}.val{font-family:monospace;color:#81c784;font-weight:bold;}"
                  ".warn{color:#ff5252;}"
                  ".grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:8px;}"
                  ".lamp{padding:10px;border-radius:5px;text-align:center;font-weight:bold;font-size:0.88em;background:#262626;color:#777;border:1px solid #333;transition:0.05s;}"
                  ".lamp-amber{background:#ffb300;color:#000;border-color:#ffe082;box-shadow:0 0 10px #ffb300;}"
                  ".lamp-red{background:#e53935;color:#fff;border-color:#ff8a80;box-shadow:0 0 10px #e53935;}"
                  ".lamp-blue{background:#039be5;color:#fff;border-color:#81d4fa;box-shadow:0 0 10px #039be5;}"
                  ".lamp-green{background:#2e7d32;color:#fff;border-color:#81c784;box-shadow:0 0 10px #2e7d32;}"
                  "</style></head><body>");

  html += F("<h2>Live System Diagnostics</h2>"
            "<div class='timer'>Portal closes in: <b id='remTime'>--</b> | Uptime: <b id='uptime'>--</b></div>"
            "<div class='nav'>"
            "<a href='/' class='btn btn-back'>&larr; Back to Settings</a>"
            "<button id='toggleBtn' class='btn btn-pause' onclick='toggleLive()'>Pause Live</button>"
            "</div>");

  // --- 1. LIVE RELAY VISUALIZER ---
  html += F("<h3>Live Relay Outputs</h3><div class='card'>"
            "<div class='grid'>"
            "<div id='rFL' class='lamp'>FRONT LEFT: OFF</div>"
            "<div id='rFR' class='lamp'>FRONT RIGHT: OFF</div>"
            "<div id='rBL' class='lamp'>REAR LEFT: OFF</div>"
            "<div id='rBR' class='lamp'>REAR RIGHT: OFF</div>"
            "</div>"
            "<div class='grid' style='grid-template-columns:1fr 1fr 1fr 1fr;margin-bottom:0;'>"
            "<div id='rHorn' class='lamp'>HORN</div>"
            "<div id='rSp1' class='lamp'>SPARE 1</div>"
            "<div id='rSp2' class='lamp'>SPARE 2</div>"
            "<div id='rSp3' class='lamp'>SPARE 3</div>"
            "</div></div>");

  // --- 2. LIVE BUTTON INPUT VISUALIZER ---
  html += F("<h3>Live Button Inputs</h3><div class='card'>"
            "<div class='grid' style='grid-template-columns:1fr 1fr 1fr 1fr;'>"
            "<div id='bLeft' class='lamp'>LEFT</div>"
            "<div id='bRight' class='lamp'>RIGHT</div>"
            "<div id='bHaz' class='lamp'>HAZARD</div>"
            "<div id='bHorn' class='lamp'>HORN</div>"
            "</div>"
            "<div class='grid' style='grid-template-columns:1fr 1fr 1fr 1fr;margin-bottom:0;'>"
            "<div id='bBrake' class='lamp'>BRAKE</div>"
            "<div id='bIgn' class='lamp'>IGNITION</div>"
            "<div id='bSp1' class='lamp'>SPARE 1</div>"
            "<div id='bSp2' class='lamp'>SPARE 2</div>"
            "</div></div>");

  auto addRow = [&](const char* label, const char* id) {
    html += "<div class='row'><span>" + String(label) + "</span><span class='val' id='" + String(id) + "'>Loading...</span></div>";
  };

  // --- 3. MEMORY & STACKS ---
  html += F("<h3>FreeRTOS Memory &amp; Task Stacks (Words Free)</h3><div class='card'>");
  addRow("Global Free Static Heap", "heap");
  addRow("processVehicleTelemetry (Core 1)", "stkTelem");
  addRow("mainInputProcessorTask (Core 1)", "stkInput");
  addRow("webConfigPortalTask (Core 1)", "stkPortal");
  addRow("turnSignalTask (Core 0)", "stkTurn");
  addRow("hazardTask (Core 0)", "stkHaz");
  addRow("brakeLightTask (Core 0)", "stkBrake");
  addRow("hornTask (Core 0)", "stkHorn");
  html += F("</div>");

  // --- 4. LIVE SYSTEM STATES ---
  html += F("<h3>Live State &amp; Output Status</h3><div class='card'>");
  addRow("Ignition Mode", "ign");
  addRow("Turn Signal Mode", "turn");
  addRow("Brake Mode", "brake");
  addRow("Hazard Mode", "hazard");
  addRow("MPU Brake Latch", "mpuLatch");
  html += F("</div>");

  // --- 5. MPU6050 TELEMETRY ---
  html += F("<h3>MPU6050 Sensor &amp; Stored Calibration</h3><div class='card'>");
  addRow("MPU6050 Hardware Status", "mpuOnline");
  addRow("MPU Auto-Brake Engine", "mpuBrakeRdy");
  addRow("Live Vehicle Pitch", "pitch");
  addRow("Filtered Forward Accel Y", "accelY");
  addRow("Calibrated Gravity Vector", "grav");
  addRow("Accel Bias [X, Y, Z]", "biasA");
  addRow("Gyro Bias [X, Y, Z]", "biasG");
  html += F("</div>");

  html += F("<script>"
            "let live = true;"
            "let busy = false;"
            "function setVal(id, text, isWarn) {"
            "  const el = document.getElementById(id);"
            "  if (!el) return;"
            "  el.textContent = text;"
            "  if (isWarn !== undefined) el.className = isWarn ? 'val warn' : 'val';"
            "}"
            "function setLamp(id, label, isOn, activeClass) {"
            "  const el = document.getElementById(id);"
            "  if (!el) return;"
            "  if (label) el.textContent = label + (isOn ? ': ON' : ': OFF');"
            "  el.className = isOn ? ('lamp ' + activeClass) : 'lamp';"
            "}"
            "function toggleLive() {"
            "  live = !live;"
            "  const btn = document.getElementById('toggleBtn');"
            "  btn.textContent = live ? 'Pause Live' : 'Resume Live';"
            "  btn.style.background = live ? '#ef6c00' : '#2e7d32';"
            "}"
            "async function fetchDiag() {"
            "  if (!live || busy) return;"
            "  busy = true;"
            "  try {"
            "    const res = await fetch('/diag-data');"
            "    const d = await res.json();"
            "    setLamp('rFL', 'FRONT LEFT', d.rFL, 'lamp-amber');"
            "    setLamp('rFR', 'FRONT RIGHT', d.rFR, 'lamp-amber');"
            "    setLamp('rBL', 'REAR LEFT', d.rBL, 'lamp-red');"
            "    setLamp('rBR', 'REAR RIGHT', d.rBR, 'lamp-red');"
            "    setLamp('rHorn', null, d.rHorn, 'lamp-blue');"
            "    setLamp('rSp1', null, d.rSp1, 'lamp-blue');"
            "    setLamp('rSp2', null, d.rSp2, 'lamp-blue');"
            "    setLamp('rSp3', null, d.rSp3, 'lamp-blue');"
            "    setLamp('bLeft', null, d.bLeft, 'lamp-green');"
            "    setLamp('bRight', null, d.bRight, 'lamp-green');"
            "    setLamp('bHaz', null, d.bHaz, 'lamp-green');"
            "    setLamp('bHorn', null, d.bHorn, 'lamp-green');"
            "    setLamp('bBrake', null, d.bBrake, 'lamp-green');"
            "    setLamp('bIgn', null, d.bIgn, 'lamp-green');"
            "    setLamp('bSp1', null, d.bSp1, 'lamp-green');"
            "    setLamp('bSp2', null, d.bSp2, 'lamp-green');"
            "    setVal('remTime', d.remTime); setVal('uptime', d.uptime);"
            "    setVal('heap', d.heap, d.heapWarn);"
            "    setVal('stkTelem', d.stkTelem, d.stkTelemWarn);"
            "    setVal('stkInput', d.stkInput, d.stkInputWarn);"
            "    setVal('stkPortal', d.stkPortal, d.stkPortalWarn);"
            "    setVal('stkTurn', d.stkTurn, d.stkTurnWarn);"
            "    setVal('stkHaz', d.stkHaz, d.stkHazWarn);"
            "    setVal('stkBrake', d.stkBrake, d.stkBrakeWarn);"
            "    setVal('stkHorn', d.stkHorn, d.stkHornWarn);"
            "    setVal('ign', d.ign); setVal('turn', d.turn);"
            "    setVal('brake', d.brake); setVal('hazard', d.hazard);"
            "    setVal('mpuLatch', d.mpuLatch);"
            "    setVal('mpuOnline', d.mpuOnline, d.mpuOnlineWarn);"
            "    setVal('mpuBrakeRdy', d.mpuBrakeRdy, d.mpuBrakeWarn);"
            "    setVal('pitch', d.pitch); setVal('accelY', d.accelY);"
            "    setVal('grav', d.grav); setVal('biasA', d.biasA); setVal('biasG', d.biasG);"
            "  } catch (e) {} finally { busy = false; }"
            "}"
            "fetchDiag();"
            "setInterval(fetchDiag, 100);"
            "</script></body></html>");

  return html;
}
/**
  * @brief Dynamically builds the responsive HTML configuration form populated with
  *        current live RAM values and displays the remaining portal time.
  */
String buildConfigHTML(String statusBanner) {
  uint32_t elapsed = millis() - portalStartTime;
  uint32_t remSec  = (elapsed < PORTAL_TIMEOUT_MS) ? (PORTAL_TIMEOUT_MS - elapsed) / 1000 : 0;

  String html = F("<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
                  "<title>Vehicle Light Controller</title><style>"
                  "body{font-family:Arial,sans-serif;background:#121212;color:#eee;margin:0;padding:15px;}"
                  "h2{color:#4db8ff;margin-bottom:5px;}h3{color:#ffcc00;border-bottom:1px solid #333;padding-bottom:4px;margin-top:20px;}"
                  ".timer{color:#aaa;font-size:0.9em;margin-bottom:15px;}"
                  ".banner{background:#1b5e20;color:#fff;padding:10px;border-radius:5px;margin-bottom:15px;font-weight:bold;}"
                  ".banner-err{background:#b71c1c;}"
                  ".row{display:flex;justify-content:space-between;align-items:center;margin:8px 0;padding:6px;background:#1e1e1e;border-radius:4px;}"
                  "label{font-size:0.95em;flex:1;padding-right:10px;}"
                  "input[type=number],input[type=text]{width:140px;padding:6px;background:#2c2c2c;color:#fff;border:1px solid #555;border-radius:4px;font-size:1em;}"
                  "input[type=checkbox]{transform:scale(1.4);margin-right:10px;}"
                  ".btns{position:sticky;bottom:0;background:#121212;padding:12px 0;display:flex;gap:10px;border-top:1px solid #333;}"
                  "button{flex:1;padding:12px;font-size:1em;font-weight:bold;border:none;border-radius:5px;cursor:pointer;}"
                  ".btn-ram{background:#0288d1;color:#fff;}.btn-flash{background:#2e7d32;color:#fff;}"
                  "</style></head><body>");

  html += "<h2>ESP32 Light Controller</h2>";
  html += "<div class='timer'>Portal auto-closes in: <b>" + String(remSec / 60) + "m " + String(remSec % 60) + "s</b></div>";
  html += "<div style='display:flex;gap:10px;margin-bottom:15px;'>"
          "<a href='/diagnostics' style='padding:8px 14px;background:#37474f;color:#fff;text-decoration:none;border-radius:5px;font-weight:bold;font-size:0.9em;'>System Diagnostics &rarr;</a>"
          "<a href='/about' style='padding:8px 14px;background:#37474f;color:#fff;text-decoration:none;border-radius:5px;font-weight:bold;font-size:0.9em;'>About &amp; Pinout &rarr;</a>"
          "</div>";

  if (statusBanner.length() > 0) {
    bool isErr = statusBanner.startsWith("ERROR");
    html += "<div class='banner" + String(isErr ? " banner-err" : "") + "'>" + statusBanner + "</div>";
  }

  html += F("<form action='/save' method='POST'>");

  auto addNum = [&](const char* label, const char* name, String val, const char* step) {
    html += "<div class='row'><label>" + String(label) + "</label>"
            "<input type='number' step='" + String(step) + "' name='" + String(name) + "' value='" + val + "' required></div>";
  };

  auto addBool = [&](const char* label, const char* name, bool checked) {
    html += "<div class='row'><label>" + String(label) + "</label>"
            "<input type='checkbox' name='" + String(name) + "' value='1'" + (checked ? " checked" : "") + "></div>";
  };
  
  auto addText = [&](const char* label, const char* name, String val, int minLen, int maxLen) {
    html += "<div class='row'><label>" + String(label) + "</label>"
            "<input type='text' name='" + String(name) + "' value='" + val + "' minlength='" + String(minLen) + "' maxlength='" + String(maxLen) + "' required></div>";
  };

  html += F("<h3>General System Settings</h3>");
  addNum("Debounce Lockout (ms)", "debounce", String(DEBOUNCE_LOCKOUT), "1");
  addBool("Use Ignition Interlock*", "ignInterlock", useIgnitionInterlock);
  addBool("Diagnostics Monitor*", "diagOn", diagnosticsOn);
  addBool("Use Brake Lights*", "useBrake", useBrakelights);
  addBool("Use MPU Auto-Brake", "useMpuBrake", useMPUBrake);
  addBool("Use MPU Auto-Turn Cancel", "useMpuTurn", useMPUTurnSignal);

  html += F("<h3>Wi-Fi Portal Settings</h3>");
  addText("Wi-Fi SSID Name", "apSsid", AP_SSID, 1, 31);
  addText("Wi-Fi Password (8+ chars)", "apPass", AP_PASS, 8, 63);
  addNum("Portal Timeout (ms)", "portalTime", String(PORTAL_TIMEOUT_MS), "1000");

  html += F("<h3>Brake Light Strobe Settings</h3>");
  addNum("Brake Flash Interval (ms)", "brkFlashInt", String(brakeFlashInterval), "1");
  addNum("Brake Flash Duration (ms)", "brkFlashDur", String(brakeFlashDuration), "1");
  addNum("Emergency Flash Interval (ms)", "emgFlashInt", String(EmergencyFlashInterval), "1");
  addNum("Emergency Flash Duration (ms)", "emgFlashDur", String(EmergencyFlashDuration), "1");

  html += F("<h3>Turn Signal Timings</h3>");
  addNum("Blink Flash Interval (ms)", "blkFlashInt", String(blinkFlashInterval), "1");
  addNum("Hold Duration Turning (ms)", "holdDurTurn", String(HoldDurationTurning), "1");
  addNum("Hold Duration Infinite (ms)", "holdDurInf", String(HoldDurationInfinite), "1");
  addNum("Lane Change Duration (ms)", "durLaneChg", String(DurationLaneChange), "1");
  addNum("Turning Flash Duration (ms)", "flashDurTurn", String(FlashDurationTurning), "1");

  html += F("<h3>Hazard Light Settings</h3>");
  addNum("Hazard Flash Interval (ms)", "hazFlashInt", String(hazardFlashInterval), "1");
  addNum("Brake Hazard Duty (%)", "brkHazPct", String(brakeHazardPercent, 1), "0.1");

  html += F("<h3>MPU6050 Calibration &amp; Mounting</h3>");
  addNum("Calibration Samples", "samples", String(samples), "1");
  addNum("Min Mount Angle X (deg)", "minMountX", String(MIN_MOUNT_ANGLE_X, 1), "0.1");
  addNum("Max Mount Angle X (deg)", "maxMountX", String(MAX_MOUNT_ANGLE_X, 1), "0.1");
  addNum("Vehicle Y Direction (1 / -1)", "vehYDir", String(VEHICLE_Y_DIRECTION, 1), "1");

  html += F("<h3>MPU6050 Filters &amp; Thresholds</h3>");
  addNum("Accel Filter Alpha", "accelAlpha", String(MPU_ACCEL_FILTER_ALPHA, 3), "0.001");
  addNum("Pitch Comp Filter Alpha", "compAlpha", String(alpha, 3), "0.001");
  addNum("Brake On Threshold (m/s&sup2;)", "brkOnThresh", String(MPU_BRAKE_ON_THRESHOLD, 2), "0.01");
  addNum("Brake Off Threshold (m/s&sup2;)", "brkOffThres", String(MPU_BRAKE_OFF_THRESHOLD, 2), "0.01");
  addNum("Brake Confirm Time (ms)", "brkConfMs", String(MPU_BRAKE_CONFIRM_MS), "1");
  addNum("Brake Release Time (ms)", "brkRelMs", String(MPU_BRAKE_RELEASE_MS), "1");
  addNum("Emergency Threshold (m/s&sup2;)", "emgThresh", String(MPU_EMERGENCY_THRESHOLD, 2), "0.01");
  addNum("Emergency Confirm Time (ms)", "emgConfMs", String(MPU_EMERGENCY_CONFIRM_MS), "1");
  addNum("Yaw Turn Cancel (deg)", "yawCancel", String(YAW_CANCEL_THRESHOLD, 1), "0.1");

  html += F("<p style='font-size:0.8em;color:#888;'>*Note: Task creation flags (Interlock, Diagnostics, Use Brakes) require Save to Flash + Reboot to take effect.</p>"
            "<div class='btns'>"
            "<button type='submit' name='action' value='ram' class='btn-ram'>Update RAM Only</button>"
            "<button type='submit' name='action' value='flash' class='btn-flash'>Save to Flash</button>"
            "</div></form>"
            "<h3>Maintenance &amp; Calibration</h3>"
            "<div style='display:flex;flex-wrap:wrap;gap:10px;margin-bottom:20px;'>"
            "<form action='/calibrate' method='POST' style='flex:1;min-width:140px;' onsubmit=\"return confirm('Ensure vehicle is parked on level ground and completely stationary. Start 3-second MPU6050 calibration?');\">"
            "<button type='submit' style='width:100%;background:#6a1b9a;color:#fff;'>Calibrate MPU6050</button>"
            "</form>"
            "<form action='/reset-defaults' method='POST' style='flex:1;min-width:140px;' onsubmit=\"return confirm('Reset ALL settings back to factory defaults and overwrite Flash?');\">"
            "<button type='submit' style='width:100%;background:#c62828;color:#fff;'>Reset to Defaults</button>"
            "</form>"
            "<form action='/reboot' method='POST' style='flex:1;min-width:140px;' onsubmit=\"return confirm('Reboot the ESP32 controller now? Unsaved RAM changes will be lost.');\">"
            "<button type='submit' style='width:100%;background:#e65100;color:#fff;'>Reboot Controller</button>"
            "</form>"
            "</div></body></html>");
  return html;
}
/**
  * @brief Builds an HTML About page detailing the multi-core architecture,
  *        hardware pin assignments, sensor mounting rules, repository links, and credits.
  */
String buildAboutHTML() {
  uint32_t elapsed = millis() - portalStartTime;
  uint32_t remSec  = (elapsed < PORTAL_TIMEOUT_MS) ? (PORTAL_TIMEOUT_MS - elapsed) / 1000 : 0;

  String html = F("<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
                  "<title>About - Vehicle Light Controller</title><style>"
                  "body{font-family:Arial,sans-serif;background:#121212;color:#eee;margin:0;padding:15px;line-height:1.45;}"
                  "h2{color:#4db8ff;margin-bottom:5px;}h3{color:#ffcc00;border-bottom:1px solid #333;padding-bottom:4px;margin-top:20px;}"
                  ".timer{color:#aaa;font-size:0.9em;margin-bottom:12px;}"
                  ".btn{display:inline-block;padding:8px 14px;background:#37474f;color:#fff;text-decoration:none;border-radius:5px;font-weight:bold;font-size:0.9em;margin-bottom:15px;}"
                  ".card{background:#1e1e1e;padding:12px;border-radius:5px;margin-bottom:12px;font-size:0.93em;}"
                  ".card p{margin:6px 0;}.card b{color:#81d4fa;}"
                  ".card a{color:#4db8ff;text-decoration:none;word-break:break-all;}"
                  ".card a:hover{text-decoration:underline;}"
                  ".row{display:flex;justify-content:space-between;padding:5px 0;border-bottom:1px solid #2a2a2a;font-size:0.9em;}"
                  ".row:last-child{border-bottom:none;}.pin{font-family:monospace;color:#81c784;font-weight:bold;}"
                  "</style></head><body>");

  html += F("<h2>ESP32 Multi-Core Vehicle Light Controller</h2>");
  html += "<div class='timer'>Portal auto-closes in: <b>" + String(remSec / 60) + "m " + String(remSec % 60) + "s</b></div>";
  html += F("<a href='/' class='btn'>&larr; Back to Settings</a>");

  // --- SYSTEM ARCHITECTURE ---
  html += F("<h3>System Architecture</h3><div class='card'>"
            "<p><b>Core 1 (Input &amp; Telemetry):</b> Processes human inputs, button debounce timing layers, hold duration upgrades, and raw MPU6050 sensor telemetry.</p>"
            "<p><b>Core 0 (Relay Actuation):</b> Non-blocking execution threads managing asynchronous blinker clocks, priority matrix masking, and multi-stage braking stroboscopic flashing patterns.</p>"
            "<p><b>Website Layer:</b> Temporary 5-minute Wi-Fi Access Point on Core 1 offering live RAM tuning, persistent NVS flash commits, remote calibration, and a 100ms AJAX diagnostics monitor.</p>"
            "<p><b>Communication Layer:</b> Core 1 communicates structural state mutations to Core 0 over thread-safe FreeRTOS message queue streams.</p>"
            "<p><b>Storage Layer:</b> Leverages <code>Preferences.h</code> to store hardware MPU sensor offset calibrations and user configuration settings inside persistent non-volatile flash memory cells.</p>"
            "<p><b>Low Power Layer:</b> Enters ultra-low power Light Sleep when Ignition (GPIO 4) is unasserted. Wakes instantly when Ignition goes LOW while still allowing Hazard Lights.</p>"
            "</div>");

  // --- SENSOR MOUNTING ---
  html += F("<h3>Sensor Mounting Requirements</h3><div class='card'>"
            "<p><b>+Y Axis:</b> Must point generally toward the front of the vehicle.</p>"
            "<p><b>X Axis:</b> Must remain across the vehicle (side-to-side).</p>"
            "<p><b>Calibration Shortcut:</b> Hold Left + Right turn buttons while turning Ignition ON, or use the remote calibration button on the main settings page while parked on level ground.</p>"
            "</div>");

  // --- HARDWARE PINOUT ---
  html += F("<h3>Hardware GPIO Pinout</h3><div class='card'>"
            "<div class='row'><span>Left / Right Turn Inputs</span><span class='pin'>GPIO 18 / GPIO 19</span></div>"
            "<div class='row'><span>Hazard / Horn Inputs</span><span class='pin'>GPIO 5 / GPIO 16</span></div>"
            "<div class='row'><span>Brake Pedal / Ignition Inputs</span><span class='pin'>GPIO 17 / GPIO 4</span></div>"
            "<div class='row'><span>Spare Inputs 1 &amp; 2</span><span class='pin'>GPIO 2 / GPIO 15</span></div>"
            "<div class='row'><span>Front Left / Right Relays</span><span class='pin'>GPIO 32 / GPIO 33</span></div>"
            "<div class='row'><span>Rear Left / Right Relays</span><span class='pin'>GPIO 25 / GPIO 26</span></div>"
            "<div class='row'><span>Horn Relay / Status LED</span><span class='pin'>GPIO 27 / GPIO 23</span></div>"
            "<div class='row'><span>Spare Relays 1, 2 &amp; 3</span><span class='pin'>GPIO 14 / 12 / 13</span></div>"
            "<div class='row'><span>MPU6050 I2C (SDA / SCL)</span><span class='pin'>GPIO 21 / GPIO 22</span></div>"
            "</div>");

  // --- REPOSITORY & LICENSING ---
  html += F("<h3>Repository &amp; Licensing</h3><div class='card'>"
            "<p><b>GitHub Repository:</b><br>"
            "<a href='https://github.com' target='_blank' rel='noopener'>"
            "https://github.com</a></p>"
            "<p><b>License Type:</b> GNU Affero General Public License v3.0 (AGPL-3.0)<br>"
            "<a href='https://github.com/blob/main/LICENSE' target='_blank' rel='noopener'>"
            "Read full license terms &rarr;</a></p>"
            "</div>");

  // --- CREDITS & EXPANSION ---
  html += F("<h3>Author &amp; Expansion Notes</h3><div class='card'>"
            "<p><b>Author:</b> Rocky Andersen</p>"
            "<p><b>Expansion Capacity:</b> Equipped with 3 spare relays and 2 spare inputs/outputs for future features and/or remote start integration.</p>"
            "</div></body></html>");

  return html;
}
/**
  * @brief Serves lightweight JSON telemetry and memory data for live AJAX updates.
  */
void handleDiagnosticsJSON() {
  uint32_t elapsed  = millis() - portalStartTime;
  uint32_t remSec   = (elapsed < PORTAL_TIMEOUT_MS) ? (PORTAL_TIMEOUT_MS - elapsed) / 1000 : 0;
  uint32_t freeHeap = ESP.getFreeHeap();

  auto getStackStr = [](TaskHandle_t handle) -> String {
    if (handle == NULL) return "\"Disabled\"";
    UBaseType_t w = uxTaskGetStackHighWaterMark(handle);
    return "\"" + String(w) + " words (" + String(w * 4) + " B)\"";
  };

  auto getStackWarn = [](TaskHandle_t handle) -> String {
    if (handle == NULL) return "false";
    return (uxTaskGetStackHighWaterMark(handle) < 150) ? "true" : "false";
  };

  String json = "{";
  json += "\"remTime\":\"" + String(remSec / 60) + "m " + String(remSec % 60) + "s\",";
  json += "\"uptime\":\"" + String(millis() / 1000) + "s\",";
  json += "\"heap\":\"" + String(freeHeap) + " bytes\",";
  json += "\"heapWarn\":" + String(freeHeap < 20000 ? "true" : "false") + ",";

  json += "\"stkTelem\":" + getStackStr(telemetryHandle) + ",\"stkTelemWarn\":" + getStackWarn(telemetryHandle) + ",";
  json += "\"stkInput\":" + getStackStr(inputTaskHandle) + ",\"stkInputWarn\":" + getStackWarn(inputTaskHandle) + ",";
  json += "\"stkPortal\":" + getStackStr(webPortalTaskHandle) + ",\"stkPortalWarn\":" + getStackWarn(webPortalTaskHandle) + ",";
  json += "\"stkTurn\":" + getStackStr(turnTaskHandle) + ",\"stkTurnWarn\":" + getStackWarn(turnTaskHandle) + ",";
  json += "\"stkHaz\":" + getStackStr(hazardTaskHandle) + ",\"stkHazWarn\":" + getStackWarn(hazardTaskHandle) + ",";
  json += "\"stkBrake\":" + getStackStr(brakeTaskHandle) + ",\"stkBrakeWarn\":" + getStackWarn(brakeTaskHandle) + ",";
  json += "\"stkHorn\":" + getStackStr(hornTaskHandle) + ",\"stkHornWarn\":" + getStackWarn(hornTaskHandle) + ",";

  json += "\"ign\":\"" + String(ignitionMode == CMD_IGNITION_ON ? "ON" : "OFF") + "\",";
  json += "\"turn\":\"" + String(signalMode == CMD_LEFT ? "LEFT" : (signalMode == CMD_RIGHT ? "RIGHT" : "OFF")) + "\",";
  json += "\"brake\":\"" + String(brakeMode == CMD_EMERGENCY_BRAKE ? "EMERGENCY" : (brakeMode == CMD_BRAKE ? "ACTIVE" : "OFF")) + "\",";
  json += "\"hazard\":\"" + String(hazardMode == CMD_HAZARD ? "ACTIVE" : "OFF") + "\",";
  json += "\"mpuLatch\":\"" + String(mpuBrakeActive ? "ENGAGED" : "IDLE") + "\",";

  json += "\"mpuOnline\":\"" + String(mpuAvailable ? "ONLINE" : "OFFLINE") + "\",";
  json += "\"mpuOnlineWarn\":" + String(!mpuAvailable ? "true" : "false") + ",";
  json += "\"mpuBrakeRdy\":\"" + String(mpuBrakeAvailable ? "READY" : "DISABLED") + "\",";
  json += "\"mpuBrakeWarn\":" + String(!mpuBrakeAvailable ? "true" : "false") + ",";
  json += "\"pitch\":\"" + String(vehiclePitchRad * (180.0 / PI), 2) + " deg\",";
  json += "\"accelY\":\"" + String(filteredVehicleAccelY, 3) + " m/s\\u00B2\",";
  json += "\"grav\":\"" + String(measuredGravity, 4) + " m/s\\u00B2\",";
  json += "\"biasA\":\"" + String(accelBiasX, 3) + ", " + String(accelBiasY, 3) + ", " + String(accelBiasZ, 3) + "\",";
  json += "\"biasG\":\"" + String(gyroBiasX, 4) + ", " + String(gyroBiasY, 4) + ", " + String(gyroBiasZ, 4) + "\",";

  // Live Hardware Relay States
  json += "\"rFL\":" + String(FrontLeft.isOn ? "true" : "false") + ",";
  json += "\"rFR\":" + String(FrontRight.isOn ? "true" : "false") + ",";
  json += "\"rBL\":" + String(BackLeft.isOn ? "true" : "false") + ",";
  json += "\"rBR\":" + String(BackRight.isOn ? "true" : "false") + ",";
  json += "\"rHorn\":" + String(HornRelay.isOn ? "true" : "false") + ",";
  json += "\"rSp1\":" + String(SpareRelay1.isOn ? "true" : "false") + ",";
  json += "\"rSp2\":" + String(SpareRelay2.isOn ? "true" : "false") + ",";
  json += "\"rSp3\":" + String(SpareRelay3.isOn ? "true" : "false") + ",";

  // Live Hardware Button Input States
  json += "\"bLeft\":" + String(Left.isPressed ? "true" : "false") + ",";
  json += "\"bRight\":" + String(Right.isPressed ? "true" : "false") + ",";
  json += "\"bHaz\":" + String(Hazard.isPressed ? "true" : "false") + ",";
  json += "\"bHorn\":" + String(Horn.isPressed ? "true" : "false") + ",";
  json += "\"bBrake\":" + String(Brake.isPressed ? "true" : "false") + ",";
  json += "\"bIgn\":" + String(Ignition.isPressed ? "true" : "false") + ",";
  json += "\"bSp1\":" + String(SpareButton1.isPressed ? "true" : "false") + ",";
  json += "\"bSp2\":" + String(SpareButton2.isPressed ? "true" : "false");

  json += "}";
  configServer.send(200, "application/json", json);
}
/**
  * @brief Parses incoming POST form variables, updates live RAM variables,
  *        and optionally commits them to NVS flash.
  */
void handleSavePage() {
  // Booleans (HTML checkboxes only send a key if checked)
  useIgnitionInterlock     = configServer.hasArg("ignInterlock");
  diagnosticsOn            = configServer.hasArg("diagOn");
  useBrakelights           = configServer.hasArg("useBrake");
  useMPUBrake              = configServer.hasArg("useMpuBrake");
  useMPUTurnSignal         = configServer.hasArg("useMpuTurn");

  // Wi-Fi Portal Settings
  if (configServer.hasArg("apSsid") && configServer.arg("apSsid").length() > 0) {
    AP_SSID = configServer.arg("apSsid");
  }
  if (configServer.hasArg("apPass") && configServer.arg("apPass").length() >= 8) {
    AP_PASS = configServer.arg("apPass");
  }
  if (configServer.hasArg("portalTime")) {
    uint32_t t = configServer.arg("portalTime").toInt();
    if (t >= 60000) PORTAL_TIMEOUT_MS = t; // Enforce 60s minimum so you don't get locked out
  }

  // Unsigned Integers
  if (configServer.hasArg("debounce"))     DEBOUNCE_LOCKOUT         = configServer.arg("debounce").toInt();
  if (configServer.hasArg("brkFlashInt"))  brakeFlashInterval       = configServer.arg("brkFlashInt").toInt();
  if (configServer.hasArg("brkFlashDur"))  brakeFlashDuration       = configServer.arg("brkFlashDur").toInt();
  if (configServer.hasArg("emgFlashInt"))  EmergencyFlashInterval   = configServer.arg("emgFlashInt").toInt();
  if (configServer.hasArg("emgFlashDur"))  EmergencyFlashDuration   = configServer.arg("emgFlashDur").toInt();
  if (configServer.hasArg("blkFlashInt"))  blinkFlashInterval       = configServer.arg("blkFlashInt").toInt();
  if (configServer.hasArg("holdDurTurn"))  HoldDurationTurning      = configServer.arg("holdDurTurn").toInt();
  if (configServer.hasArg("holdDurInf"))   HoldDurationInfinite     = configServer.arg("holdDurInf").toInt();
  if (configServer.hasArg("durLaneChg"))   DurationLaneChange       = configServer.arg("durLaneChg").toInt();
  if (configServer.hasArg("flashDurTurn")) FlashDurationTurning     = configServer.arg("flashDurTurn").toInt();
  if (configServer.hasArg("hazFlashInt"))  hazardFlashInterval      = configServer.arg("hazFlashInt").toInt();
  if (configServer.hasArg("brkConfMs"))    MPU_BRAKE_CONFIRM_MS     = configServer.arg("brkConfMs").toInt();
  if (configServer.hasArg("brkRelMs"))     MPU_BRAKE_RELEASE_MS     = configServer.arg("brkRelMs").toInt();
  if (configServer.hasArg("emgConfMs"))    MPU_EMERGENCY_CONFIRM_MS = configServer.arg("emgConfMs").toInt();

  // Signed Integers
  if (configServer.hasArg("samples"))      samples                  = configServer.arg("samples").toInt();

  // Floats
  if (configServer.hasArg("brkHazPct"))    brakeHazardPercent       = configServer.arg("brkHazPct").toFloat();
  if (configServer.hasArg("minMountX"))    MIN_MOUNT_ANGLE_X        = configServer.arg("minMountX").toFloat();
  if (configServer.hasArg("maxMountX"))    MAX_MOUNT_ANGLE_X        = configServer.arg("maxMountX").toFloat();
  if (configServer.hasArg("vehYDir"))      VEHICLE_Y_DIRECTION      = configServer.arg("vehYDir").toFloat();
  if (configServer.hasArg("accelAlpha"))   MPU_ACCEL_FILTER_ALPHA   = configServer.arg("accelAlpha").toFloat();
  if (configServer.hasArg("compAlpha"))    alpha                    = configServer.arg("compAlpha").toFloat();
  if (configServer.hasArg("brkOnThresh"))  MPU_BRAKE_ON_THRESHOLD   = configServer.arg("brkOnThresh").toFloat();
  if (configServer.hasArg("brkOffThres"))  MPU_BRAKE_OFF_THRESHOLD  = configServer.arg("brkOffThres").toFloat();
  if (configServer.hasArg("emgThresh"))    MPU_EMERGENCY_THRESHOLD  = configServer.arg("emgThresh").toFloat();
  if (configServer.hasArg("yawCancel"))    YAW_CANCEL_THRESHOLD     = configServer.arg("yawCancel").toFloat();

  String action = configServer.arg("action");
  String msg    = "";

  if (action == "flash") {
    saveSystemConfig();
    msg = "SUCCESS: Settings applied to RAM and saved permanently to Flash!";
    Serial.println("[PORTAL] Updated RAM and committed to Flash.");
  } else {
    msg = "SUCCESS: Running RAM variables updated (Temporary until reboot).";
    Serial.println("[PORTAL] Updated running RAM variables only.");
  }

  configServer.send(200, "text/html", buildConfigHTML(msg));
}
/**
  * @brief Safely grounds all relays, sends a confirmation response, and restarts the ESP32.
  */
void handleRebootPage() {
  Serial.println("[PORTAL] Remote reboot requested via Web Portal. Restarting ESP32...");

  String html = F("<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
                  "<meta http-equiv='refresh' content='5;url=/'>"
                  "<title>Rebooting...</title><style>"
                  "body{font-family:Arial,sans-serif;background:#121212;color:#eee;text-align:center;padding:40px 15px;}"
                  "h2{color:#ffb300;}p{color:#aaa;line-height:1.5;}"
                  "</style></head><body>"
                  "<h2>Rebooting Controller...</h2>"
                  "<p>All relays have been safely grounded.<br>"
                  "If the Wi-Fi portal is set to auto-launch, this page will reload in 5 seconds.<br>"
                  "Otherwise, turn the ignition off and back on (or hold Hazard + Horn) to reopen the portal.</p>"
                  "</body></html>");

  configServer.send(200, "text/html", html);
  vTaskDelay(pdMS_TO_TICKS(500)); // Allow TCP packet to finish transmitting to browser

  // Ground all physical relay outputs before resetting
  for (int i = 0; i < NUM_RELAYS; i++) {
    digitalWrite(relays[i]->value, LOW);
    relays[i]->isOn = false;
  }

  Serial.flush();
  ESP.restart();
}
/**
  * @brief Restores all user settings to factory defaults and reloads the page.
  */
void handleResetDefaultsPage() {
  resetSystemConfigDefaults();
  configServer.send(200, "text/html", buildConfigHTML("SUCCESS: All settings reset to factory defaults and saved to Flash!"));
}
/**
  * @brief Safely pauses competing sensor/relay tasks, executes static 6-axis MPU6050
  *        calibration, resets live filters, and resumes tasks.
  */
void handleCalibratePage() {
  if (!mpuAvailable) {
    configServer.send(200, "text/html", buildConfigHTML("ERROR: MPU6050 sensor not detected on I2C bus."));
    return;
  }

  // Notify telemetry task to pause I2C polling via queue
  bool calibState = true;
  xQueueOverwrite(calibrationQueue, &calibState);
  vTaskDelay(pdMS_TO_TICKS(15)); // Allow any active I2C read to complete

  bool success = performSensorCalibration();

  if (success) {
    vehiclePitchRad       = staticMountPitchRad;
    filteredVehicleAccelY = 0.0;
    mpuBrakeActive        = false;
    mpuBrakeCandidate     = false;
    mpuEmergencyCandidate = false;
    accumulatedYaw        = 0.0;
    turnInitiated         = false;
  }

  // Resume telemetry I2C polling via queue
  calibState = false;
  xQueueOverwrite(calibrationQueue, &calibState);

  if (success) {
    configServer.send(200, "text/html", buildConfigHTML("SUCCESS: MPU6050 calibrated and saved to Flash!"));
  } else {
    configServer.send(200, "text/html", buildConfigHTML("ERROR: Calibration failed. Ensure vehicle is stationary."));
  }
}
/**
  * @brief Serves the About and Hardware Pinout page.
  */
void handleAboutPage() {
  configServer.send(200, "text/html", buildAboutHTML());
}
/**
  * @brief Serves the live system diagnostics page.
  */
void handleDiagnosticsPage() {
  configServer.send(200, "text/html", buildDiagnosticsHTML());
}
/**
  * @brief Serves the main configuration page.
  */
void handleRootPage() {
  configServer.send(200, "text/html", buildConfigHTML());
}
/**
  * @brief Core 1 FreeRTOS task that hosts the Wi-Fi AP and Web Server for 5 minutes,
  *        then shuts down Wi-Fi and deletes itself to save power and memory.
  * 
  * @param pvParameters Standard untyped FreeRTOS task argument block pointer (Unused).
  */
void webConfigPortalTask(void* pvParameters) {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID.c_str(), AP_PASS.c_str());
  portalStartTime = millis();

  configServer.on("/", HTTP_GET, handleRootPage);
  configServer.on("/save", HTTP_POST, handleSavePage);
  configServer.on("/calibrate", HTTP_POST, handleCalibratePage);
  configServer.on("/reset-defaults", HTTP_POST, handleResetDefaultsPage);
  configServer.on("/reboot", HTTP_POST, handleRebootPage);
  configServer.on("/diagnostics", HTTP_GET, handleDiagnosticsPage);
  configServer.on("/diag-data", HTTP_GET, handleDiagnosticsJSON);
  configServer.on("/about", HTTP_GET, handleAboutPage);
  configServer.begin();

  Serial.println("=========================================================");
  Serial.printf("[PORTAL] Wi-Fi Config Portal Active for 5 Minutes!\n");
  Serial.printf("[PORTAL] SSID: %s | Password: %s\n", AP_SSID.c_str(), AP_PASS.c_str());
  Serial.print("[PORTAL] Open browser to: http://");
  Serial.println(WiFi.softAPIP());
  Serial.println("=========================================================");

  while (millis() - portalStartTime < PORTAL_TIMEOUT_MS) {
    configServer.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // 5 minutes elapsed: Cleanly tear down server and disable Wi-Fi radio
  Serial.println("[PORTAL] 5-minute window expired. Shutting down Wi-Fi portal...");
  configServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);

  webPortalTaskHandle = NULL;
  vTaskDelete(NULL); // Self-terminate task cleanly
}
/**
  * @brief Safely launches the 5-minute Wi-Fi Configuration Portal if not already active.
  */
void startWebPortal() {
  if (webPortalTaskHandle == NULL) {
    xTaskCreatePinnedToCore(
      webConfigPortalTask,
      "webConfigPortal",
      4096,
      NULL,
      1,
      &webPortalTaskHandle,
      1 // Pinned to Core 1 so Core 0 relay timing remains deterministic
    );
    Serial.println("[PORTAL] Ignition ON detected: Launching 5-minute Wi-Fi portal.");
  } else {
    // If already running, reset the 5-minute timer
    portalStartTime = millis();
    Serial.println("[PORTAL] Portal already active: Resetting 5-minute timer.");
  }
}
/**
  * @brief Forcefully stops the Wi-Fi portal early (e.g., when Ignition turns OFF before 5 mins).
  */
void stopWebPortal() {
  if (webPortalTaskHandle != NULL) {
    Serial.println("[PORTAL] Ignition OFF detected: Shutting down Wi-Fi portal early...");
    configServer.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);

    TaskHandle_t tempHandle = webPortalTaskHandle;
    webPortalTaskHandle = NULL;
    vTaskDelete(tempHandle);
  }
}