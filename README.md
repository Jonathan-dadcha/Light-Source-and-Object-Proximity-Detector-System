# MSP430 Embedded Radar & Light Tracking System

**Authors:** Adar Shapira & Yehonatan Dadaka  
**Institution:** Ben-Gurion University of the Negev  
**Course:** Digital Computer Systems

## 📖 Abstract
This project is a comprehensive Embedded System designed on the **TI MSP430G2553**. It integrates hardware actuation, sensor data acquisition, and a custom flash-based file system. 

The system operates in multiple modes to detect objects (using Ultrasound) and track light sources (using dual LDRs). It features a bi-directional UART communication protocol connecting the MCU to a rich Python-based PC GUI, allowing for real-time data visualization (Radar Plots), remote control, and file management.

## 🚀 Key Features

### 📡 Radar & Sensing
* **Ultrasonic Sweep:** Scans 180° using a Servo motor and HC-SR04 sensor to map object distances.
* **Light Tracking:** Uses differential measurements from two LDRs to pinpoint light sources in the environment.
* **Combo Sweep:** Simultaneously maps physical objects and light intensity.
* **Calibration:** Automated calibration routine for LDR sensors with interpolation logic on the PC side.

### 💾 Custom Flash File System
* **In-Memory Storage:** A custom file system implemented in the MCU's Flash memory.
* **Capabilities:** Supports `LS` (List), `PUT` (Upload), `GET` (Download), `DEL` (Delete), and `FORMAT`.
* **File Types:** Stores text files and executable scripts directly on the chip.
* **LCD Browser:** View and navigate files directly on the onboard 16x2 LCD using push buttons.

### 🖥️ PC Interface (GUI)
* **Control Dashboard:** Python-based GUI (PySimpleGUI) to trigger FSM states.
* **Data Visualization:** Real-time Polar (Radar) plots using **Matplotlib**.
* **File Manager:** Drag-and-drop interface to upload files from PC to MCU.

## 🛠️ Hardware & Pin Configuration

**MCU:** MSP430G2553  
**Logic Voltage:** 3.3V

| Component | Function | MSP430 Pin | Notes |
| :--- | :--- | :--- | :--- |
| **UART** | RX (Input) | **P1.1** | 9600 Baud, 8N1 |
| **UART** | TX (Output) | **P1.2** | |
| **Servo** | PWM Signal | **P2.1** | Timer A1.1 (50Hz) |
| **Ultrasonic** | Trigger | **P2.3** | 10µs pulse |
| **Ultrasonic** | Echo | **P2.2** | Timer A1 Capture (Use voltage divider if 5V) |
| **LDR Left** | Analog In | **P1.5** | ADC Channel A5 |
| **LDR Right** | Analog In | **P1.7** | ADC Channel A7 |
| **LCD** | RS | **P1.3** | |
| **LCD** | Enable | **P1.0** | |
| **LCD** | Data (D4-D7) | **P2.4 - P2.7** | 4-bit Mode |
| **Buttons** | PB0 (Next) | **P2.0** | Active Low (Pull-up) |
| **Buttons** | PB1 (Select) | **P1.6** | Active Low (Pull-up) |

## 🏗️ Software Architecture

The firmware is built using a layered HAL/BSP approach for portability and modularity:

1.  **BSP (Board Support Package):** Low-level pin muxing, clock setup (1MHz), and interrupt vector definitions.
2.  **HAL (Hardware Abstraction Layer):** Drivers for Servo, LCD, UART, and ADC. Abstracts register manipulation.
3.  **API (Application Programming Interface):** High-level logic (e.g., `servo_ultra_sweep`, `api_file_put`). Contains the math for data averaging and protocol handling.
4.  **APP (Application):** The `main.c` loop containing the Finite State Machine (FSM) that switches between Idle, Sweep, and File modes based on UART commands.

## ⚙️ Installation & Usage

### Firmware (MCU)
1.  Open the project in **Code Composer Studio (CCS)**.
2.  Ensure the `include` paths match your directory structure.
3.  Build and Flash to the MSP430G2553.

### PC GUI
1.  Install Python 3.x.
2.  Install dependencies:
    ```bash
    pip install -r gui/requirements.txt
    ```
3.  Run the interface:
    ```bash
    python gui/GUI.py
    ```

### Operation
1.  **Connect:** Select the MSP430 COM port in the GUI and click "Connect".
2.  **Control Tab:** Use "US 180° Sweep" to generate a radar map of the room.
3.  **Files Tab:** Upload a text file to the MCU, then use the buttons on the board (PB0/PB1) to view it on the LCD.

## 📂 Protocol Details
Communication relies on a custom ASCII-based protocol over UART.
* **Command Example:** `STATE,1` (Start Ultrasonic Sweep)
* **Data Example:** `DIST,90,120` (At 90°, distance is 120cm)
* **File Transfer:** Uses a Chunked `PUT` mechanism with Checksums to ensure data integrity over UART.

## 📄 License
This project is open-source. Please attribute the authors if used for educational purposes.
