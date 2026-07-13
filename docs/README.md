# Dual YD-RP2040 CRSF & MAVLink Multiplexer, Switcher, VRX Controller & Antenna Tracker

This system utilizes two YD-RP2040 boards connected via high-speed UART to multiplex MAVLink, CRSF, and configuration commands into a single physical serial link. It also features a PC-side graphical configuration software, antenna tracking, and a VRX module controller.

---

## Architecture Overview

```
+------------------------------------+          High-Speed UART Link           +------------------------------------+
|        BOARD 1 (Ground Station)    | <=====================================> |         BOARD 2 (RF Switcher)      |
+------------------------------------+                (UART 1)                 +------------------------------------+
  | (USB CDC 0)  | (USB CDC 1)  | (I2C)                                          | (UART 0)            | (PIO SoftUART)
  v              v              v                                                v                     v
Windows COM     PC GUI         VRX Module                                     JR Module 1           JR Module 2
(MAVLink)       Control        (I2C address 0x35)                             (CRSF+MAVLink)        (CRSF Only)
```

---

## Wiring & Pin Definitions

### Board 1: Ground Station (YD-RP2040)
* **High-Speed Inter-Board UART (UART 1)**:
  * **TX**: `GPIO 4` -> Connect to Board 2 `GPIO 5` (RX)
  * **RX**: `GPIO 5` -> Connect to Board 2 `GPIO 4` (TX)
* **Servos**:
  * **Azimuth Servo (Pan)**: `GPIO 14` (PWM Output)
  * **Elevation Servo (Tilt)**: `GPIO 15` (PWM Output)
* **VRX Module Control (I2C 0)**:
  * **SDA**: `GPIO 16` -> Connect to VRX I2C SDA
  * **SCL**: `GPIO 17` -> Connect to VRX I2C SCL
  * *Note: Pull-up resistors (typically 4.7kΩ) are recommended on both I2C lines.*

### Board 2: RF Module Switcher (YD-RP2040)
* **High-Speed Inter-Board UART (UART 1)**:
  * **TX**: `GPIO 4` -> Connect to Board 1 `GPIO 5` (RX)
  * **RX**: `GPIO 5` -> Connect to Board 1 `GPIO 4` (TX)
* **JR Module 1 (UART 0) - CRSF + MAVLink**:
  * **TX**: `GPIO 0` -> Connect to JR Module 1 RX
  * **RX**: `GPIO 1` -> Connect to JR Module 1 TX
  * **Power Enable**: `GPIO 12` (Active High)
* **JR Module 2 (PIO SoftUART) - CRSF Only**:
  * **TX**: `GPIO 8` -> Connect to JR Module 2 RX
  * **RX**: `GPIO 9` -> Connect to JR Module 2 TX
  * **Power Enable**: `GPIO 13` (Active High)

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
   pip install pyserial
   ```
3. Run the application:
   ```bash
   python control_program/main.py
   ```
4. Connect Board 1 to your computer via USB. Windows will display two Virtual COM ports:
   * **COM Port 1 (CDC 0)**: Connect your Ground Station software (e.g., Mission Planner, QGroundControl) directly to this port to stream MAVLink.
   * **COM Port 2 (CDC 1)**: Select this COM port in the PC Configurator application and click **Connect**.
