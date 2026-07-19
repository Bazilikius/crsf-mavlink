# Dual YD-RP2040 CRSF & MAVLink Multiplexer, Switcher, VRX Controller & Antenna Tracker

This system utilizes two YD-RP2040 boards running MicroPython, connected via a high-speed UART1 inter-board link at **400,000 baud**, to multiplex MAVLink, CRSF, and configuration commands dynamically. It features a PC-side configuration software with a MAVLink UDP bridge, PC-processed antenna tracking, and a VRX module controller.

---

## Architecture Overview

```
+------------------------------------+          High-Speed UART Link           +------------------------------------+
|        BOARD 1 (Ground Station)    | <=====================================> |         BOARD 2 (RF Switcher)      |
+------------------------------------+                (UART 1)                 +------------------------------------+
                  | (USB VCP)                                                            | (UART 0)            | (PIO SoftUART)
                  v                                                                      v                     v
            PC Configurator                                                           JR Module 1           JR Module 2
       (MAVLink UDP Port 19415/14556)                                                 (CRSF+MAVLink)        (CRSF Only)
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
* **VRX SYNTH (I2C 0)**:
  * **SDA**: `GPIO 16` -> Connect to VRX SDA
  * **SCL**: `GPIO 17` -> Connect to VRX SCL
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

## Automatic Startup on Re-power / Power-On

Both boards are designed to automatically run and operate on power-on or re-power. To configure this:
1. Save `board1_ground/main.py` onto Board 1's root directory as **`main.py`**.
2. Save `board2_rf/main.py` onto Board 2's root directory as **`main.py`**.
3. Once named `main.py`, MicroPython will automatically execute the multiplexer and switcher loops immediately upon power-up/re-power.

---

## PC-Based Tracker Servo Driving
All MAVLink telemetry processing and tracking calculations (Azimuth/Elevation angles) are performed entirely on the PC control program side. This greatly simplifies Pico's firmware and optimizes performance. The PC computes the relative angles and drives Board 2's tracker servos dynamically using standard `0x70` command packets.

---

## Running the PC Control Program

The PC Configurator application allows real-time switching between JR module modes, calibrating tracker servos, setting home coordinates, and configuring video receiver (VRX) frequencies.

### Real-Time Status Indicators (Visual Panel)
The PC Configurator features status indicators on the connection panel:
* **RF Board ONLINE/OFFLINE**: Live heartbeat detection. Board 2 periodically sends heartbeats over UART1 to Board 1, which communicates this state to the PC to verify the RF Board connection.
* **MAV OK / MAV: NO DATA**: Verifies that MAVLink telemetry is actively transferring through the system.

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
   * **MAVP2P Routing**: Use `Start MAVP2P` / `Stop MAVP2P` buttons to manually manage background routing to UDP port `19415` (PC Configurator) and port `14556` (GCS like Mission Planner or QGroundControl) over 400,000 baud!
