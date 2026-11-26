# ESP32 Multi-Core Vehicle Light Controller

A deterministic, real-time automotive lighting controller built using the **Arduino IDE** for the **Aceirmc ESP32-WROOM-32E 8-Channel Relay Development Board** paired with an optional **KEAcvise GY-521 (MPU6050) 6-Axis IMU Module**. Designed to separate high-speed lighting control from input scanning, motion telemetry, and wireless configuration across the ESP32's dual cores.

## Overview

Modern vehicle lighting requires immediate, deterministic response for safety-critical signals (braking, deceleration, turn indicators, and hazards) alongside customizable lighting timings. By pinning time-critical relay switching to **Core 0** while running input scanning, MPU6050 telemetry, power supervision, and the Wi-Fi configuration portal on **Core 1**, this firmware ensures network traffic and sensor math never delay critical vehicle light responses.

## Key Features

* **Dual-Core FreeRTOS Task Separation:**
  * **Core 0 (`PRO_CPU` — Real-Time Relay Actuation):** Dedicated exclusively to asynchronous relay tasks (turn signals, brake strobing, hazard blending, and horn) plus a Core 0 hardware watchdog heartbeat.
  * **Core 1 (`APP_CPU` — Inputs, Telemetry & Web Portal):** Handles hardware ISR button debouncing, hold-duration turn signal tiers, MPU6050 I2C sensor fusion, ignition power state supervision, NVS storage (`Preferences`), and the temporary Wi-Fi configuration/OTA web portal.
* **IMU-Assisted Motion & Braking Detection (Optional):**
  * **Auto-Brake & Emergency Strobe:** Uses gravity-compensated forward deceleration to trigger standard brake lights or high-speed emergency brake strobing—even during engine braking or downshifting without touching the pedal.
  * **Auto-Turn Signal Cancel:** Integrates Z-axis gyroscope yaw rate to automatically cancel turn signals after completing a corner.
* **Multi-Tiered Turn Signal Controls:** Supports short micro-clicks (lane-change timeout), medium holds (intersection timeout), persistent long holds (infinite latch), and double-tap manual cancellation.
* **8-Channel High-Current Relay Switching:** Controls 5 primary vehicle circuits (Front Left, Front Right, Rear Left/Brake, Rear Right/Brake, Horn) plus 3 spare relays (`GPIO 14`, `GPIO 12`, `GPIO 13`) for future expansion (fog lights, DRLs, off-road light bars, or rock lights).
* **Ultra-Low Power Light Sleep:** When Ignition Interlock is enabled and the key is off, the supervisor puts the MPU6050 into standby, grounds all relays, and enters ESP32 Light Sleep—waking instantly on Ignition (`GPIO 4`) or Hazard (`GPIO 5`) activation.
* **Captive Wi-Fi Configuration & OTA Portal:** Launches a 5-minute Wi-Fi Access Point (`Vehicle_Light_Config`) for live RAM tuning, persistent NVS flash saves, remote MPU6050 calibration, real-time AJAX diagnostics, and Over-The-Air (`.bin`) firmware updates.
* **Fail-Safe Hardware Watchdog:** Monitors both Core 0 and Core 1 tasks (`3s` timeout) and executes an IRAM fail-safe handler (`esp_task_wdt_isr_user_handler`) to immediately ground all relays if a fault or panic occurs.

## Architecture

| Core | Assigned FreeRTOS Tasks | Priority | Role |
| :--- | :--- | :--- | :--- |
| **Core 0 (`PRO_CPU`)** | `turnSignalTask`, `hazardTask`, `brakeLightTask`, `hornTask`, `core0WdtTask` | Normal (`1`) | Deterministic relay flasher clocks, brake/hazard duty-cycle blending, and Core 0 WDT heartbeat |
| **Core 1 (`APP_CPU`)** | `processSystemInterlockSupervisor` | High (`3`) | Manages active/hazard/light-sleep power states and OTA task suspension |
| **Core 1 (`APP_CPU`)** | `mainInputProcessorTask` | Medium-High (`2`) | Evaluates debounced switch edges, hold tiers, and portal launch combos |
| **Core 1 (`APP_CPU`)** | `processVehicleTelemetry`, `webConfigPortalTask` | Normal (`1`) | MPU6050 I2C complementary filter, yaw integration, HTTP server, and NVS saves |
| **Core 1 (`APP_CPU`)** | `systemDiagnosticsTask` | Low (`0`) | Optional serial stack high-water mark and free heap monitor |

*Inter-core state sharing uses thread-safe, single-item FreeRTOS queues (`xQueueOverwrite`) and IRAM-safe hardware interrupts.*

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
| **Button 3** | `GPIO 5` | Hazard | Latching toggle switch (also wakes from sleep) |
| **Button 4** | `GPIO 16` | Horn | Momentary push-button |
| **Button 5** | `GPIO 17` | Brake Pedal *(Optional)* | Mechanical brake light switch |
| **Button 6** | `GPIO 4` | Ignition *(Optional)* | Key-on sense (also wakes from sleep) |
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
* To calibrate on a level, stationary surface, either click **Calibrate MPU6050** in the web portal or hold **Left + Right** turn buttons while turning the Ignition ON. All 4 corner lights will flash 3 times upon success.

## Getting Started (Arduino IDE)

### 1. Arduino IDE & Board Setup
1. Install the latest Arduino IDE.
2. Open **File > Preferences** and add the Espressif ESP32 board manager URL to *Additional Boards Manager URLs*:
   ```text
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
3. Go to **Tools > Board > Boards Manager**, search for **esp32** by Espressif Systems, and click **Install**.
4. Select **Tools > Board > ESP32 Arduino > ESP32 Dev Module** (ensure a partition scheme with OTA support, such as *Default 4MB with spiffs*, is selected).

### 2. Required Arduino Libraries
Install the following libraries via **Sketch > Include Library > Manage Libraries**:
* **Adafruit MPU6050** (by Adafruit)
* **Adafruit Unified Sensor** (by Adafruit)

*(All other dependencies—`WiFi`, `WebServer`, `Update`, `Preferences`, and `Wire`—are built into the ESP32 Arduino core.)*

### 3. Flashing the Firmware
Because the Aceirmc ESP32 8-Channel Relay board uses an external serial header rather than a built-in USB port:
1. Connect your 3.3V USB-to-TTL adapter (`TX` -> `RX`, `RX` -> `TX`, `GND` -> `GND`, `3V3` -> `3V3`).
2. Hold down the onboard **`IO0` (BOOT)** button, press and release **`EN` (RESET)**, then release **`IO0`** to enter bootloader mode.
3. Select the correct COM port under **Tools > Port** and click **Upload** in the Arduino IDE.
4. Press **`EN` (RESET)** once uploading finishes to boot the dual-core firmware.

### 4. Accessing the Wi-Fi Configuration Portal
* **At Boot / Ignition On:** The Wi-Fi Access Point starts automatically for 5 minutes (default SSID: `Vehicle_Light_Config`, Password: `12345678`).
* **Manual Trigger (when Ignition Interlock is disabled):** Hold **Hazard + Horn** simultaneously for 2 seconds (all 4 corner lights will flash twice to confirm).
* Connect to the Wi-Fi network and navigate to **`http://192.168.4.1`** in your browser.

## Roadmap
* Add running lights (DRLs) tied to Ignition ON state
* ~~Add OTA to web configuration~~ *(Completed)*
* Add remote start integration:
  * Tachometer / RPM pulse input to verify engine running state
  * Secondary ESP32 wireless remote fob (press-and-hold to start / press-and-hold to stop)

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