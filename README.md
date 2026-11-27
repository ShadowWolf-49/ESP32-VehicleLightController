# ESP32 Multi-Core Vehicle Light Controller

A deterministic, real-time automotive lighting controller built using the **Arduino IDE** for the **Aceirmc ESP32-WROOM-32E 8-Channel Relay Development Board** paired with an optional **KEAcvise GY-521 (MPU6050) 6-Axis IMU Module**. Designed to separate high-speed lighting control from input scanning, motion telemetry, and wireless configuration across the ESP32's dual cores.

## Overview

Modern vehicle lighting requires immediate, deterministic response for safety-critical signals (braking, deceleration, turn indicators, and hazards) alongside customizable lighting timings. By pinning time-critical relay switching to **Core 0** while running input scanning, MPU6050 telemetry, power supervision, and the Wi-Fi configuration portal on **Core 1**, this firmware ensures network traffic and sensor math never delay critical vehicle light responses.

## Key Features

* **Dual-Core FreeRTOS Task Separation:**
  * **Core 0 (`PRO_CPU` — Real-Time Relay Actuation):** Dedicated exclusively to asynchronous relay tasks (turn signals, brake strobing, hazard blending, and horn) plus a Core 0 hardware watchdog heartbeat.
  * **Core 1 (`APP_CPU` — Inputs, Telemetry & Web Portal):** Handles hardware ISR button debouncing, hold-duration turn signal tiers, MPU6050 I2C sensor fusion, ignition power state supervision, NVS storage (`Preferences`), and the temporary Wi-Fi configuration/OTA web portal.
* **Smart Priority Masking & Brake/Hazard Blending:**
  * **Turn + Brake Priority:** When braking while a turn signal is active, the turning side flashes normally while the opposite rear light maintains solid brake illumination.
  * **Hazard + Brake Duty-Cycle Blending:** When braking with Hazards active, the rear lights pulse using a configurable duty cycle (`70%` ON / `30%` OFF per hazard cycle by default) while the front lights continue flashing symmetrically.
* **IMU-Assisted Motion & Braking Detection (Optional):**
  * **Auto-Brake & Emergency Strobe:** Uses gravity-compensated forward deceleration to trigger standard brake lights (`-1.75 m/s²` default) or high-speed emergency brake strobing (`-5.0 m/s²` default)—even during engine braking or downshifting without touching the pedal.
  * **Auto-Turn Signal Cancel:** Integrates Z-axis gyroscope yaw rate to automatically cancel turn signals after completing a corner (`15°` threshold default).
* **Multi-Tiered Turn Signal Controls:** Supports short micro-clicks (`5s` lane-change timeout), medium holds (`1.5s` hold for `20s` intersection timeout), persistent long holds (`5s` hold for infinite latch), and double-tap manual cancellation.
* **8-Channel High-Current Relay Switching:** Controls 5 primary vehicle circuits (Front Left, Front Right, Rear Left/Brake, Rear Right/Brake, Horn) plus 3 spare relays (`GPIO 14`, `GPIO 12`, `GPIO 13`) for future expansion (fog lights, DRLs, off-road light bars, or rock lights).
* **Ultra-Low Power Light Sleep (Optional Ignition Interlock):** Disabled by default (`useIgnitionInterlock = false`) for standalone operation. When enabled in the web portal, turning the key off (`GPIO 4`) puts the MPU6050 into standby, grounds all relays, and enters ESP32 Light Sleep—waking immediately when Ignition (`GPIO 4`) goes LOW or when Hazard (`GPIO 5`) is pressed (which immediately latches Hazard mode upon wakeup).
* **Captive Wi-Fi Configuration & Safe OTA Portal:** Launches a 5-minute Wi-Fi Access Point (`Vehicle_Light_Config`) with wildcard DNS captive portal redirection (`http://192.168.4.1`) for live RAM tuning, persistent NVS flash saves, remote MPU6050 calibration, `100ms` real-time AJAX diagnostics, and Over-The-Air (`.bin`) firmware updates with `0xE9` image validation and automatic relay grounding during flash writes.
* **Fail-Safe Hardware Watchdog:** Monitors both Core 0 and Core 1 tasks (`3s` timeout) and executes an IRAM-resident fail-safe handler (`esp_task_wdt_isr_user_handler`) to immediately ground all relays if a fault or panic occurs.

## Architecture

| Core | Assigned FreeRTOS Tasks | Priority | Role |
| :--- | :--- | :--- | :--- |
| **Core 0 (`PRO_CPU`)** | `turnSignalTask`, `hazardTask`, `brakeLightTask`, `hornTask`, `core0WatchdogTask` | Normal (`1`) | Deterministic relay flasher clocks, brake/hazard duty-cycle blending, and Core 0 WDT heartbeat |
| **Core 1 (`APP_CPU`)** | `processSystemInterlockSupervisor` | High (`3`) | Manages active/hazard/light-sleep power states and OTA task suspension *(when Interlock is enabled)* |
| **Core 1 (`APP_CPU`)** | `mainInputProcessorTask` | Medium-High (`2`) | Evaluates debounced switch edges, hold tiers, and portal launch combos |
| **Core 1 (`APP_CPU`)** | `processVehicleTelemetry`, `webConfigPortalTask` | Normal (`1`) | MPU6050 I2C complementary filter, yaw integration, Captive DNS/HTTP server, and NVS saves |
| **Core 1 (`APP_CPU`)** | `systemDiagnosticsTask` | Low (`0`) | Optional serial stack high-water mark and free heap monitor |

*Inter-core state sharing uses thread-safe, single-item FreeRTOS queues (`xQueueOverwrite`), FreeRTOS Event Groups (`systemEventGroup`), and IRAM-safe hardware interrupts.*

## Repository Structure

```text
├── ESP32-VehicleLightController.ino  # Entry point: boot fail-safes, WDT init, queues, and FreeRTOS task creation
├── config.h / config.cpp             # Hardware GPIO definitions, FreeRTOS handles, and default tuning parameters
├── inputs.h / inputs.cpp             # IRAM button debounce ISR, hold-tier logic, and portal combo detection
├── mpu_sensor.h / mpu_sensor.cpp     # MPU6050 calibration, complementary pitch filter, auto-brake, and yaw cancel
├── relays.h / relays.cpp             # Core 0 relay state machines, priority masking, and IRAM WDT fail-safe grounding
├── storage.h / storage.cpp           # ESP32 NVS (Preferences) save, load, and factory-reset routines
├── supervisor.h / supervisor.cpp     # Core 1 Ignition/Hazard power state machine, Light Sleep, and OTA interlocks
└── web_portal.h / web_portal.cpp     # Captive DNS + Wi-Fi web server, live AJAX diagnostics, calibration, and OTA updater
```

## Hardware Components

* **Main Controller Board:** Aceirmc ESP32-WROOM-32E 8-Channel 5V Relay Development Board
* **Inertial Sensor Module (Optional):** KEAcvise GY-521 MPU6050 6-Axis Accelerometer & Gyroscope (I2C)
* **Power Connection:** Direct `7V–30V DC` terminal block wired to fused 12V/24V vehicle power *(do not connect AC mains in a vehicle installation)*
* **Programming Adapter:** USB-to-TTL Serial Adapter (3.3V logic) connected to the board's `TX`, `RX`, `GND`, and `3V3` programming header

## Pinout Configuration

*Note: Verify relay and input GPIO mappings against your specific board silkscreen and `config.cpp`.*

### 1. Onboard 8-Channel Relay Mapping

| Relay Channel | ESP32 GPIO | Assigned Vehicle Circuit | Terminal Contact |
| :--- | :--- | :--- | :--- |
| **Relay 1** | `GPIO 32` | Front Left Light | `COM` / `NO` |
| **Relay 2** | `GPIO 33` | Front Right Light | `COM` / `NO` |
| **Relay 3** | `GPIO 25` | Rear Left / Brake Light | `COM` / `NO` |
| **Relay 4** | `GPIO 26` | Rear Right / Brake Light | `COM` / `NO` |
| **Relay 5** | `GPIO 27` | Horn | `COM` / `NO` |
| **Relay 6** | `GPIO 14` | Spare Relay 1 | `COM` / `NO` |
| **Relay 7** | `GPIO 12` | Spare Relay 2 | `COM` / `NO` |
| **Relay 8** | `GPIO 13` | Spare Relay 3 | `COM` / `NO` |

*Note: Onboard Status LED is mapped to `GPIO 23`.*

### 2. Switch & Button Input Mapping (Active-LOW with Internal Pull-Ups)

| Input Channel | ESP32 GPIO | Assigned Vehicle Circuit | Switch Type / Notes |
| :--- | :--- | :--- | :--- |
| **Button 1** | `GPIO 18` | Left Turn | Momentary push-button (tap/hold tiers) |
| **Button 2** | `GPIO 19` | Right Turn | Momentary push-button (tap/hold tiers) |
| **Button 3** | `GPIO 5` | Hazard | Momentary push-button (wakes from sleep & latches Hazard) |
| **Button 4** | `GPIO 16` | Horn | Momentary push-button |
| **Button 5** | `GPIO 17` | Brake Pedal *(Optional)* | Mechanical brake light switch |
| **Button 6** | `GPIO 4` | Ignition *(Optional)* | Key-on sense (requires **Use Ignition Interlock** enabled) |
| **Button 7** | `GPIO 2` | Spare Input 1 *(Optional)* | Reserved for future expansion |
| **Button 8** | `GPIO 15` | Spare Input 2 *(Optional)* | Reserved for future expansion |

### 3. GY-521 MPU6050 IMU Wiring (I2C — Optional)

| GY-521 Pin | ESP32 Pin | Notes |
| :--- | :--- | :--- |
| **VCC** | `3V3` | Use 3.3V output from the ESP32 header |
| **GND** | `GND` | Common ground |
| **SDA** | `GPIO 21` | Default ESP32 I2C Data |
| **SCL** | `GPIO 22` | Default ESP32 I2C Clock |
| **AD0** | `GND` | Sets I2C address to `0x68` |

**Sensor Mounting & Calibration:**
* Mount the GY-521 with the **+Y axis** pointing toward the front of the vehicle (or set `VEHICLE_Y_DIRECTION = -1.0` in the web portal if mounted backward) and the **X axis** across the vehicle side-to-side within `±45°` pitch.
* On first boot (when uncalibrated in NVS), the controller automatically runs a 300-sample static calibration.
* To recalibrate on a level, stationary surface, either click **Calibrate MPU6050** in the web portal or—when Ignition Interlock is enabled—hold **Left + Right** turn buttons while turning Ignition ON (`GPIO 4`). All 4 corner lights will flash 3 times upon success.

## Getting Started (Arduino IDE)

### 1. Arduino IDE & Board Setup
1. Install the latest Arduino IDE.
2. Open **File > Preferences** and add the Espressif ESP32 board manager URL to *Additional Boards Manager URLs*:
   ```text
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
3. Go to **Tools > Board > Boards Manager**, search for **esp32** by Espressif Systems, and click **Install**.
4. Select **Tools > Board > ESP32 Arduino > ESP32 Dev Module** (ensure a partition scheme with OTA support, such as *Default 4MB with spiffs* or *Minimal SPIFFS*, is selected).

### 2. Required Arduino Libraries
Install the following libraries via **Sketch > Include Library > Manage Libraries**:
* **Adafruit MPU6050** (by Adafruit)
* **Adafruit Unified Sensor** (by Adafruit)

*(All other dependencies—`WiFi`, `DNSServer`, `WebServer`, `Update`, `Preferences`, and `Wire`—are built into the ESP32 Arduino core.)*

### 3. Flashing the Firmware
Because the Aceirmc ESP32 8-Channel Relay board uses an external serial header rather than a built-in USB port:
1. Connect your 3.3V USB-to-TTL adapter (`TX` -> `RX`, `RX` -> `TX`, `GND` -> `GND`, `3V3` -> `3V3`).
2. Hold down the onboard **`IO0` (BOOT)** button, press and release **`EN` (RESET)**, then release **`IO0`** to enter bootloader mode.
3. Select the correct COM port under **Tools > Port** and click **Upload** in the Arduino IDE.
4. Press **`EN` (RESET)** once uploading finishes to boot the dual-core firmware.

### 4. Accessing the Captive Wi-Fi Configuration Portal
* **Standalone Mode (`Use Ignition Interlock` OFF — Factory Default):**
  * The Wi-Fi Access Point launches automatically for 5 minutes at power-on (default SSID: `Vehicle_Light_Config`, Password: `12345678`).
  * To re-open the portal later without rebooting, hold **Hazard + Horn** simultaneously for **2 seconds** (all 4 corner lights will flash twice to confirm).
* **Ignition Interlock Mode (`Use Ignition Interlock` ON):**
  * Enable **Use Ignition Interlock** in the web portal, click **Save to Flash**, and **Reboot Controller**.
  * The Wi-Fi portal will launch automatically for 5 minutes every time Ignition (`GPIO 4`) is switched ON, and shut down immediately when Ignition is switched OFF.
* Connect to the `Vehicle_Light_Config` Wi-Fi network—your device should automatically open the captive portal popup, or you can navigate directly to **`http://192.168.4.1`** in your browser.
* *Mobile OTA Tip:* Some phone captive portal popups restrict `<input type="file">` uploads. If selecting a `.bin` file does not work inside the popup, open **`http://192.168.4.1`** directly in Safari or Chrome.

## Configuration Parameters Reference

All 31 runtime parameters can be adjusted live in RAM or saved permanently to Non-Volatile Storage (`Preferences` namespace `"SysConfig"`) via the web portal.

### General System Settings
| Web Portal Label | C++ Variable (`config.cpp`) | NVS Key (`SysConfig`) | Type | Default | Description & Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Debounce Lockout (ms)** | `DEBOUNCE_LOCKOUT` | `debounce` | `uint32_t` | `15` | Minimum time (ms) between valid switch edges in `buttonISR`. |
| **Use Ignition Interlock*** | `useIgnitionInterlock` | `ignInterlock` | `bool` | `false` | Enables `GPIO 4` key-sense supervision and Light Sleep *(requires Save + Reboot)*. |
| **Diagnostics Monitor*** | `diagnosticsOn` | `diagOn` | `bool` | `false` | Enables serial FreeRTOS stack & heap logging every 5s *(requires Save + Reboot)*. |
| **Use Brake Lights*** | `useBrakelights` | `useBrake` | `bool` | `true` | Enables `brakeLightTask` and pedal/MPU braking logic *(requires Save + Reboot)*. |
| **Use MPU Auto-Brake** | `useMPUBrake` | `useMpuBrake` | `bool` | `true` | Allows MPU6050 deceleration to trigger standard & emergency brake lights. |
| **Use MPU Auto-Turn Cancel** | `useMPUTurnSignal` | `useMpuTurn` | `bool` | `true` | Allows MPU6050 Z-axis gyro yaw integration to auto-cancel active turn signals. |

### Wi-Fi Portal Settings
| Web Portal Label | C++ Variable (`config.cpp`) | NVS Key (`SysConfig`) | Type | Default | Description & Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Wi-Fi SSID Name** | `AP_SSID` | `apSsid` | `String` | `"Vehicle_Light_Config"` | Access Point network name (`1–31` characters). |
| **Wi-Fi Password (8+ chars)** | `AP_PASS` | `apPass` | `String` | `"12345678"` | WPA2 Access Point password (`8–63` characters). |
| **Portal Timeout (ms)** | `PORTAL_TIMEOUT_MS` | `portalTime` | `uint32_t` | `300000` | Auto-shutdown timer for the Wi-Fi portal (`300000` = 5 min; minimum `60000` ms). |

### Brake Light Strobe Settings
| Web Portal Label | C++ Variable (`config.cpp`) | NVS Key (`SysConfig`) | Type | Default | Description & Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Brake Flash Interval (ms)** | `brakeFlashInterval` | `brkFlashInt` | `uint32_t` | `140` | Strobe toggle period (ms) during initial standard braking. |
| **Brake Flash Duration (ms)** | `brakeFlashDuration` | `brkFlashDur` | `uint32_t` | `1000` | Total strobe duration (ms) before switching to solid brake illumination. |
| **Emergency Flash Interval (ms)** | `EmergencyFlashInterval` | `emgFlashInt` | `uint32_t` | `70` | Rapid strobe toggle period (ms) during hard emergency braking. |
| **Emergency Flash Duration (ms)** | `EmergencyFlashDuration` | `emgFlashDur` | `uint32_t` | `1000` | Total rapid strobe duration (ms) before holding solid brake illumination. |

### Turn Signal Timings
| Web Portal Label | C++ Variable (`config.cpp`) | NVS Key (`SysConfig`) | Type | Default | Description & Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Blink Flash Interval (ms)** | `blinkFlashInterval` | `blkFlashInt` | `uint32_t` | `400` | ON/OFF flash half-period (ms) for turn indicators. |
| **Hold Duration Turning (ms)** | `HoldDurationTurning` | `holdDurTurn` | `uint32_t` | `1500` | Button hold threshold (ms) to upgrade from lane-change to intersection mode. |
| **Hold Duration Infinite (ms)** | `HoldDurationInfinite` | `holdDurInf` | `uint32_t` | `5000` | Button hold threshold (ms) to latch turn signal indefinitely (`0xFFFFFFFF`). |
| **Lane Change Duration (ms)** | `DurationLaneChange` | `durLaneChg` | `uint32_t` | `5000` | Auto-cancel timeout (ms) for a short micro-click (`< 1500` ms hold). |
| **Turning Flash Duration (ms)** | `FlashDurationTurning` | `flashDurTurn` | `uint32_t` | `20000` | Auto-cancel timeout (ms) for a medium hold (`1500–4999` ms). |

### Hazard Light Settings
| Web Portal Label | C++ Variable (`config.cpp`) | NVS Key (`SysConfig`) | Type | Default | Description & Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Hazard Flash Interval (ms)** | `hazardFlashInterval` | `hazFlashInt` | `uint32_t` | `450` | ON/OFF flash half-period (ms) for hazard lights (`900` ms full cycle). |
| **Brake Hazard Duty (%)** | `brakeHazardPercent` | `brkHazPct` | `float` | `70.0` | Percentage of each full hazard cycle that rear lights stay ON when braking. |

### MPU6050 Calibration & Mounting
| Web Portal Label | C++ Variable (`config.cpp`) | NVS Key (`SysConfig`) | Type | Default | Description & Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Calibration Samples** | `samples` | `samples` | `int` | `300` | Number of `10` ms sensor samples averaged during static calibration (`300` = ~3s). |
| **Min Mount Angle X (deg)** | `MIN_MOUNT_ANGLE_X` | `minMountX` | `float` | `-45.0` | Minimum allowable static mounting pitch angle (degrees) during calibration. |
| **Max Mount Angle X (deg)** | `MAX_MOUNT_ANGLE_X` | `maxMountX` | `float` | `45.0` | Maximum allowable static mounting pitch angle (degrees) during calibration. |
| **Vehicle Y Direction (1 / -1)** | `VEHICLE_Y_DIRECTION` | `vehYDir` | `float` | `1.0` | Set to `1.0` if MPU6050 `+Y` points forward, or `-1.0` if mounted backward. |

### MPU6050 Filters & Thresholds
| Web Portal Label | C++ Variable (`config.cpp`) | NVS Key (`SysConfig`) | Type | Default | Description & Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Accel Filter Alpha** | `MPU_ACCEL_FILTER_ALPHA` | `accelAlpha` | `float` | `0.15` | Exponential moving-average smoothing factor (`0.0–1.0`) for forward Y acceleration. |
| **Pitch Comp Filter Alpha** | `alpha` | `compAlpha` | `float` | `0.98` | Complementary filter weight blending gyro-X rate (`0.98`) vs. accel pitch (`0.02`). |
| **Brake On Threshold (m/s²)** | `MPU_BRAKE_ON_THRESHOLD` | `brkOnThresh` | `float` | `-1.75` | Forward deceleration threshold (`m/s²`) required to trigger standard auto-braking. |
| **Brake Off Threshold (m/s²)** | `MPU_BRAKE_OFF_THRESHOLD` | `brkOffThres` | `float` | `-0.80` | Hysteresis recovery threshold (`m/s²`) required to release auto-brake lights. |
| **Brake Confirm Time (ms)** | `MPU_BRAKE_CONFIRM_MS` | `brkConfMs` | `uint32_t` | `120` | Sustained deceleration time (ms) required before activating auto-brake lights. |
| **Brake Release Time (ms)** | `MPU_BRAKE_RELEASE_MS` | `brkRelMs` | `uint32_t` | `250` | Sustained recovery time (ms) above `brkOffThres` before turning brake lights off. |
| **Emergency Threshold (m/s²)** | `MPU_EMERGENCY_THRESHOLD` | `emgThresh` | `float` | `-5.0` | Hard deceleration threshold (`m/s²`) to trigger rapid emergency brake strobing. |
| **Emergency Confirm Time (ms)** | `MPU_EMERGENCY_CONFIRM_MS` | `emgConfMs` | `uint32_t` | `80` | Sustained hard deceleration time (ms) required to filter potholes/bumps. |
| **Yaw Turn Cancel (deg)** | `YAW_CANCEL_THRESHOLD` | `yawCancel` | `float` | `15.0` | Cumulative heading change (degrees) after turn initiation (`>= 10°/s`) to auto-cancel signal. |

*\*Task creation flags (`useIgnitionInterlock`, `diagnosticsOn`, `useBrakelights`) are read at boot in `setup()`, so changes require **Save to Flash** followed by **Reboot Controller**.*

## Roadmap
* Add running lights (DRLs) tied to Ignition ON state
* ~~Add OTA to web configuration~~ *(Completed)*
* Add remote start integration:
  * Tachometer / RPM pulse input to verify engine running state
  * Secondary ESP32 wireless remote fob (press-and-hold to start / press-and-hold to stop)
* Clean up code

## Automotive Safety Disclaimer

> **WARNING:** Modifying vehicle lighting systems is subject to local, state, and federal road safety regulations (such as DOT/SAE or ECE standards). Ensure all custom lighting modes, strobes, and auxiliary light bars comply with local laws before operating on public roads. Always mount the relay board inside a non-conductive, vibration-resistant enclosure, install appropriate inline automotive fuses on all 12V/24V supply lines, and verify fail-safe relay states before driving.

## Contributing

Contributions, bug reports, and feature requests are welcome. All contributions submitted to this repository are governed by the same **AGPL-3.0** license.

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/imu-braking-threshold`)
3. Commit your changes (`git commit -m 'Add configurable G-force braking threshold'`)
4. Push to the branch (`git push origin feature/imu-braking-threshold`)
5. Open a Pull Request

## License & Attribution

**ESP32 Multi-Core Vehicle Light Controller**  
Copyright (C) 2026 Rocky Andersen

This firmware is licensed under the **GNU Affero General Public License v3.0 (AGPL-3.0)**. See the [LICENSE](./LICENSE) file for the full legal text.

### Summary of Terms
* **Keep Source Open:** If you modify this firmware, distribute it on hardware, or connect it to a network/web interface, you must release the complete corresponding source code under the same AGPL-3.0 license.
* **No Hardware Lockdowns:** Commercial hardware running this firmware must allow end-users to flash modified versions of the code.
* **Preserve Credit:** All original copyright notices, author attributions, and license texts must remain intact in any copies or derivative works.

### Commercial Licensing
If you are a business or hardware manufacturer looking to integrate this firmware into a proprietary, closed-source commercial product without AGPL-3.0 obligations, please contact Rocky Andersen (`rlandersen@net-shadow.net`) to inquire about a private commercial license.
