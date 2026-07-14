# Dual YD-RP2040 CRSF & MAVLink Multiplexer, Switcher, VRX Controller & Antenna Tracker

This system utilizes two YD-RP2040 boards running MicroPython, connected via a high-speed UART1 inter-board link, to multiplex MAVLink, CRSF, and configuration commands dynamically. It features a PC-side configuration software with a MAVLink UDP bridge, antenna tracking, and a VRX module controller.

---

## Architecture Overview

```
+------------------------------------+          High-Speed UART Link           +------------------------------------+
|        BOARD 1 (Ground Station)    | <=====================================> |         BOARD 2 (RF Switcher)      |
+------------------------------------+                (UART 1)                 +------------------------------------+
                  | (USB VCP)                                                            | (UART 0)            | (PIO SoftUART)
                  v                                                                      v                     v
            PC Configurator                                                           JR Module 1           JR Module 2
            (MAVLink UDP Port 14550)                                                  (CRSF+MAVLink)        (CRSF Only)
```

---

## Wiring & Pin Definitions

### Board 1: Ground Station (YD-RP2040)
* **High-Speed Inter-Board UART (UART 1)**:
  * **TX**: `GPIO 4` -> Connect to Board 2 `GPIO 5` (RX)
  * **RX**: `GPIO 5` -> Connect to Board 2 `GPIO 4` (TX)
* **Radiomaster TX16S Connection (UART 0)**:
  * **TX**: `GPIO 0` -> Connect to TX16S RX
  * **RX**: `GPIO 1` -> Connect to TX16S TX
* **Potentiometers**:
  * **Azimuth Pot (Manual override)**: `GPIO 26` (ADC 0)
  * **Elevation Pot (Manual override)**: `GPIO 27` (ADC 1)
* **Cam Switch**:
  * **Cam Select**: `GPIO 18` -> High = VRX Camera active, Low = Analog Camera active
* **OLED Display & VRX SYNTH (I2C 0)**:
  * **SDA**: `GPIO 16` -> Connect to OLED SDA and VRX SDA
  * **SCL**: `GPIO 17` -> Connect to OLED SCL and VRX SCL
  * *Note: Pull-up resistors (typically 4.7kΩ) are recommended on both I2C lines.*

### Board 2: RF Module Switcher (YD-RP2040)
* **High-Speed Inter-Board UART (UART 1)**:
  * **TX**: `GPIO 4` -> Connect to Board 1 `GPIO 5` (RX)
  * **RX**: `GPIO 5` -> Connect to Board 1 `GPIO 4` (TX)
* **Servos**:
  * **Azimuth Servo (Pan, 360°)**: `GPIO 14` (PWM Output)
  * **Elevation Servo (Tilt, 180°)**: `GPIO 15` (PWM Output)
* **JR Module 1 - Dual Interface (Hardware UART 0 + PIO Soft-UART)**:
  * **MAVLink (Hardware UART 0)**:
    * **TX**: `GPIO 0` -> Connect to JR Module 1 MAVLink RX
    * **RX**: `GPIO 1` -> Connect to JR Module 1 MAVLink TX
  * **CRSF (PIO Soft-UART, State Machine 0/1)**:
    * **TX**: `GPIO 10` -> Connect to JR Module 1 CRSF RX
    * **RX**: `GPIO 11` -> Connect to JR Module 1 CRSF TX
  * **Power Enable**: `GPIO 12` (Controlled via 50Hz PWM: 2000us is ON, 1000us is OFF to drive RC electronic switches)
* **JR Module 2 - CRSF Only (PIO Soft-UART, State Machine 2/3)**:
  * **TX**: `GPIO 8` -> Connect to JR Module 2 CRSF RX
  * **RX**: `GPIO 9` -> Connect to JR Module 2 CRSF TX
  * **Power Enable**: `GPIO 13` (Controlled via 50Hz PWM: 2000us is ON, 1000us is OFF to drive RC electronic switches)

---

## Operating Modes

1. **Mode 1: Single JR Module 1 Active (CRSF + MAVLink)**:
   * JR Module 1 is powered on and handles both MAVLink telemetry and CRSF control commands.
   * JR Module 2 is powered down.
2. **Mode 2: Single JR Module 2 Active (CRSF Only)**:
   * JR Module 2 is powered on and handles CRSF.
   * JR Module 1 is powered down.
3. **Mode 3: Simultaneous/Split Operation**:
   * Both JR Modules are powered on.
   * JR Module 1 handles MAVLink telemetry exclusively.
   * JR Module 2 handles CRSF control commands exclusively.

---

## Running the PC Control Program

The PC Configurator application allows real-time switching between JR module modes, calibrating tracker servos, setting home coordinates, and configuring video receiver (VRX) frequencies.

### Setup Instructions
1. Ensure Python 3 is installed.
2. Install dependencies:
   ```bash
   pip install pyserial tkintermapview
   ```
3. Run the application:
   * GUI Mode (requires display):
     ```bash
     python control_program/main.py
     ```
   * Headless CLI Mode:
     ```bash
     python control_program/main.py --cli
     ```
4. Connect Board 1 to your computer via USB. Select its USB COM port in the PC Configurator and click **Connect**.
5. Once connected:
   * **GCS Integration**: The PC Configurator runs a **Transparent MAVLink UDP Proxy Server on Port 14550**. Open your Ground Control Station (e.g., Mission Planner or QGroundControl), select connection type **UDP**, set port to **14550**, and click Connect. Telemetry streams seamlessly while avoiding Windows COM port conflicts!
