# ESP32 Multi-Core Vehicle Light Controller

A deterministic, real-time automotive lighting controller built using the **Arduino IDE** for the **Aceirmc ESP32-WROOM-32E 8-Channel Relay Development Board** paired with a **KEAcvise GY-521 (MPU6050) 6-Axis IMU Module**. Designed to separate high-speed lighting control and motion sensing from wireless connectivity and system management across the ESP32's dual cores.

## Overview

Modern vehicle lighting requires zero-lag response for safety-critical signals (braking, deceleration, turn indicators, reverse) alongside customizable auxiliary lighting modes. By pinning time-critical relay switching **Core 0** while running Wi-Fi configuration, input processing and telemetry on **Core 1**, this firmware ensures network traffic never delays critical vehicle light responses.

## Key Features

* **Dual-Core FreeRTOS Task Separation:**
  * **Core 1 (Real-Time Control & IMU Engine):** Dedicated exclusively to high-speed I2C polling of the GY-521 MPU6050, deceleration/braking detection, vehicle input reading, and web configuration interface
  * **Core 0 (Storage & System Management):** Handles all relay outputs, and non-volatile settings storage (`Preferences` / NVS).
* **IMU-Assisted Motion & Braking Detection:** Uses the 3-axis accelerometer and 3-axis gyroscope on the MPU6050 to detect hard braking (inertia-triggered brake light strobing/override), and turn-rate feedback to disable turn signals after turning.
* **8-Channel High-Current Relay Switching:** Controls up to 8 independent 12V vehicle lighting circuits (brake lights, turn signals, reverse lights) via onboard SPDT relays (COM, NO, NC). 3 spare relays for possible  (fog lights, DRLs, off-road light bars, and rock lights)
* **Wide-Range Automotive Power Input:** Leverages the board's onboard DC switching regulator (`7V–30V DC` input) to run directly off a 12V/24V vehicle electrical system.
* **Fail-Safe Hardware Watchdog:** Monitors Core 1 execution and forces all relays to a safe default state if a sensor or task fault occurs.

## Architecture

| Core | Assigned FreeRTOS Tasks | Priority |
| :--- | :--- | :--- |
| **Core 1 (`APP_CPU`)** | MPU6050 I2C sensor fusion, deceleration/tilt thresholds, vehicle input polling, Wi-Fi / BLE stack, Web UI / REST API server
| **Core 0 (`PRO_CPU`)** | 8-channel relay state machine | High (Real-Time) NVS flash configuration saves | Normal / Background |

*Inter-core state sharing uses thread-safe FreeRTOS queues and mutex-protected structs to prevent race conditions.*

## Hardware Components

* **Main Controller Board:** Aceirmc US AC/DC Power Supply ESP32-WROOM-32E Development Board (8-Channel 5V Relay Module)
* **Inertial Sensor Module:** KEAcvise GY-521 MPU6050 6-Axis Accelerometer & Gyroscope (I2C)
* **Power Connection:** Direct `7V–30V DC` terminal block wired to fused 12V vehicle ignition/battery power *(do not connect AC mains in a vehicle installation)*
* **Programming Adapter:** USB-to-TTL Serial Adapter (3.3V logic) connected to the board's `TX`, `RX`, `GND`, and `3V3` programming header

## Pinout Configuration

*Note: On standard ESP32-WROOM-32E 8-channel relay boards, Relays 1–8 are mapped to GPIOs `32, 33, 25, 26, 27, 14, 12, 13` (verify against your specific board silkscreen or `config.h`).*

### 1. Onboard 8-Channel Relay Mapping

| Relay Channel | ESP32 GPIO | Assigned Vehicle Lighting Circuit | Terminal Contact |
| :--- | :--- | :--- | :--- |
| **Relay 1** | `GPIO 32` | Front Left Light | `COM` / `NO` |
| **Relay 2** | `GPIO 33` | Front Right Light | `COM` / `NO` |
| **Relay 3** | `GPIO 25` | Rear Left/BrakeLight | `COM` / `NO` |
| **Relay 4** | `GPIO 26` | Rear Right/BrakeLight | `COM` / `NO` |
| **Relay 5** | `GPIO 27` | Horn | `COM` / `NO` |
| **Relay 6** | `GPIO 14` | Spare | `COM` / `NO` |
| **Relay 7** | `GPIO 12` | Spare | `COM` / `NO` |
| **Relay 8** | `GPIO 13` | Spare | `COM` / `NO` |

| Button Channel | ESP32 GPIO | Assigned Vehicle  Circuit |
| :--- | :--- | :--- |
| **Button 1** | `GPIO 18` | Left Turn |
| **Button 2** | `GPIO 19` | Right Turn |
| **Button 3** | `GPIO 5` | Hazard |
| **Button 4** | `GPIO 16` | Horn |
| **Button 5** | `GPIO 17` | Brake Optional |
| **Button 6** | `GPIO 4` | Ignition Optional |
| **Button 7** | `GPIO 2` | Spare Optional |
| **Button 8** | `GPIO 15` | Spare Optional |


### 2. GY-521 MPU6050 IMU Wiring (I2C) | Iptional

| GY-521 Pin | ESP32 Pin | Notes |
| :--- | :--- | :--- |
| **VCC** | `3V3` | Use 3.3V output from the ESP32 header |
| **GND** | `GND` | Common ground |
| **SDA** | `GPIO 21` | Default ESP32 I2C Data |
| **SCL** | `GPIO 22` | Default ESP32 I2C Clock |
| **AD0** | `GND` | Sets I2C address to `0x68` |

## Getting Started (Arduino IDE)

### 1. Arduino IDE & Board Setup
1. Install the latest [Arduino IDE](https://arduino.cc).
2. Open **File > Preferences** and add the Espressif ESP32 board manager URL to *Additional Boards Manager URLs*:
   ```text
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
3. Go to **Tools > Board > Boards Manager**, search for **esp32** by Espressif Systems, and click **Install**.
4. Select **Tools > Board > ESP32 Arduino > ESP32 Dev Module**.

### 2. Required Arduino Libraries
Install the following libraries via **Sketch > Include Library > Manage Libraries**:
* **Adafruit MPU6050** (by Adafruit)
* **Adafruit Unified Sensor** (by Adafruit)
* **ArduinoJson** (by Benoit Blanchon — if using web/API configuration)

### 3. Flashing the Firmware
Because the Aceirmc ESP32 8-Channel Relay board uses an external serial header rather than a built-in USB port:
1. Connect your 3.3V USB-to-TTL adapter (`TX` -> `RX`, `RX` -> `TX`, `GND` -> `GND`, `3V3` -> `3V3`).
2. Hold down the onboard **`IO0` (BOOT)** button, press and release **`EN` (RESET)**, then release **`IO0`** to enter bootloader mode.
3. Select the correct COM port under **Tools > Port** and click **Upload** in the Arduino IDE.
4. Press **`EN` (RESET)** once uploading finishes to boot the dual-core firmware.

## Roadmap
* Add running lights to turn on when Ignition is on
* Add remote start features
   * Input RPM readings to make sure running
   * Use Second ESP-32 for remote start button
       * Press and hold once / Start
       * Press and hold again / Stop

## Automotive Safety Disclaimer

> **WARNING:** Modifying vehicle lighting systems is subject to local, state, and federal road safety regulations (such as DOT/SAE or ECE standards). Ensure all custom lighting modes, strobes, and auxiliary light bars comply with local laws before operating on public roads. Always mount the relay board inside a non-conductive, vibration-resistant enclosure, install appropriate inline automotive fuses on all 12V supply lines, and verify fail-safe relay states before driving.

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
If you are a business or hardware manufacturer looking to integrate this firmware into a proprietary, closed-source commercial product without AGPL-3.0 obligations, please contact Rocky Andersen [rlandersen@net-shadow.net] to inquire about a private commercial license.
