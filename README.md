# Vehicle CAN Diagnostic Tool (Ather)

An open-source diagnostic utility designed to interface with the CAN bus of Ather electric scooters. This tool listens to, decodes, and logs real-time vehicle telemetry data to help users understand their vehicle's performance, battery health, and system diagnostics.

## Features
* **Real-Time CAN Sniffing:** Intercept raw CAN frames directly from the vehicle's diagnostic port.
* **Data Decoding:** Translate raw hexadecimal IDs and payloads into readable metrics (e.g., RPM, battery voltage, throttle position).
* **Live Logging:** Export CAN traffic for offline analysis.

## Hardware Prerequisites
To build the physical scanner, you will need:
* **Microcontroller:** ESP32-C3 or ESP32-S3 development board.
* **CAN Transceiver:** SN65HVD230 (3.3V compatible) or similar CAN transceiver module.
* **Connecting Cable:** OBD-II or custom Ather diagnostic port connector.
* Jumper wires and a breadboard/custom PCB.

## Basic Wiring Guide

| ESP32 (C3/S3/WROOM) Pin | SN65HVD230 Pin | Function |
| :--- | :--- | :--- |
| 3V3 | 3V3 | Power |
| GND | GND | Ground |
| TX (e.g., GPIO 10/01/23 ) | CTX | CAN Transmit |
| RX (e.g., GPIO 20/13/22 ) | CRX | CAN Receive |

*Connect the CAN High (CAN H) and CAN Low (CAN L) from the transceiver to the corresponding pins on the Ather diagnostic port.*

## Installation & Setup
1. Clone this repository:
   `git clone https://github.com/kukun7610/Vehicle_CAN_Diagnostic_tool.git`
2. Open the project in your preferred IDE (PlatformIO or Arduino IDE).
3. Install the required CAN bus libraries (e.g., `ESP32-TWAI-CAN` or standard `CAN.h`).
4. Flash the code to your ESP32-TWAI Supported board.
5. Connect the hardware to your vehicle and monitor the serial output.

## Disclaimer
This project is for educational and diagnostic purposes only. It is not affiliated with or endorsed by Ather Energy. Interfacing with your vehicle's CAN bus carries inherent risks; use this tool at your own discretion.
