## main.c - include the main FSM states and the program entry point.
Handles UART RX interrupts, parses incoming commands from the PC, and triggers transitions between FSM states 
(Idle, Ultrasonic Sweep, Ultrasonic Angle, LDR Step Print, LDR Sweep, File Mode).

Functions:
- void main(void): Program entry point. Initializes system, peripherals, and enters FSM loop.
- static void tx_putc(char c): Sends a single character over UART.
- static void tx_write(const char* s): Sends a string over UART.
- static void tx_line(const char* s): Sends a string followed by newline.
- static void delay_ms(unsigned int ms): Delay function in milliseconds (busy-wait).

## api.c - include the application-level functions on the high level.
Provides services for sensors, servo motor control, ultrasonic distance measurements, and light detection by wrapping the HAL functions.

Functions:
- void api_servo_set_angle(uint8_t deg): Sets the servo motor angle using PWM.
- bool api_ultra_get_cm(uint16_t* cm): Gets distance from ultrasonic sensor (in cm).
- void api_ldr_step_print(uint16_t vcc): Reads and prints a single step from LDR sensors.
- void servo_ultra_sweep(void): Performs a 180° sweep using servo + ultrasonic sensor.
- void servo_ldr_sweep(uint16_t vcc): Performs a 180° sweep with two LDR sensors.

## hal.c - include the HAL functions on the lower level.
Implements low-level access to peripherals (UART, ADC, Timers, GPIO, ultrasonic trigger/echo), supporting the API layer.

Functions:
- void hal_ultra_init_polling(void): Initializes ultrasonic sensor in polling mode.
- void hal_ldr_begin(void): Initializes LDR sensors and ADC.
- void hal_uart_putu(unsigned int num): Sends unsigned integer via UART.
- Additional helper functions directly accessing MSP430 registers.

## flash.c - include the flash functions for File Mode operations.
Implements storing, reading, and executing scripts/files in the MCU Flash segments without overwriting program code.

Functions:
- void flash_write(const char* fname, const char* data): Writes data to flash under given filename.
- void flash_read(const char* fname): Reads data from flash file.
- void flash_delete(const char* fname): Deletes a file from flash.
- void flash_list(void): Lists all files stored in flash.

## bsp.c - include the initialization of the modules and board configuration.
Configures system clock, UART, timers, ADC, and defines the pin mapping for external devices (Ultrasonic, Servo, LDR).

Functions:
- void clock_init_1mhz(void): Initializes system clock at 1 MHz.
- void uart_init_9600(void): Initializes UART at 9600 baud.
- void sysConfig(void): General system configuration for MCU peripherals.

## all headers - include externed functions, global variables, and constant definitions.
Used to declare APIs across the different abstraction layers (APP, API, HAL, BSP).

Files:
- app.h: Application-level prototypes and constants.
- api.h: API function declarations.
- hal.h: HAL function declarations for peripherals.
- bsp.h: BSP constants (pin mapping, hardware definitions).
- flash.h: Flash memory management structures and declarations.

## GUI.py - GUI.py is a Python file built to connect the MSP430 microcontroller to a PC-side interface.
The PC-side GUI controls the FSM states, sends files, and displays incoming data from the MCU.

The GUI includes the following libraries:
PySimpleGUI, time, serial, tkinter, mouse, os, threading, binascii

Functions:
- main(): Launches the GUI, initializes serial connection, and event loop.
- send_state(n): Sends FSM state selector command to MCU.
- send_file(path): Transfers file contents to MCU over UART.
- receive_data(): Receives and displays data coming from MCU.
- helper functions for GUI event handling and threading.

## NOTE
Further description, FSM diagrams, and design details are provided in the full project report.

## Hardware Connections (Physical Wiring)

> All logic signals are 3.3V. **Common GND is mandatory** between MSP430 and all peripherals.
> If your ultrasonic module's ECHO is 5V, use a resistor divider or a 3.3V‑safe module.

### UART (PC ↔ MSP430)
| Function | MSP430 Pin | Notes |
|---|---|---|
| UCA0RXD | **P1.1** | USB‑UART RX (from PC) |
| UCA0TXD | **P1.2** | USB‑UART TX (to PC) |
| Baud    | —         | 9600‑8‑N‑1 |

### LCD 1602A — 4‑bit interface
| LCD Signal | MSP430 Pin | Notes |
|---|---|---|
| RS  | **P1.3** | Register Select |
| RW  | **P1.4** (held LOW) | Write‑only (may be tied directly to GND) |
| E   | **P1.0** | Enable (disable on‑board LED jumper if needed) |
| D4  | **P2.4** | Data (high nibble) |
| D5  | **P2.5** | 〃 |
| D6  | **P2.6** | 〃 |
| D7  | **P2.7** | 〃 |
| VSS/VDD/VO | — | VSS→GND, VDD→3.3V, VO via contrast potentiometer (~10 kΩ) |
| BL+ / BL-  | — | Backlight per module spec (typically via resistor) |

### Push Buttons
| Button | MSP430 Pin | Mode | Notes |
|---|---|---|---|
| PB0 | **P2.0** | Input with internal Pull‑Up, IRQ on falling edge | Press shorts to GND |
| PB1 | **P1.6** | Input with internal Pull‑Up, IRQ | Press shorts to GND |

### Servo (PWM control)
| Signal | MSP430 Pin | Timer | Notes |
|---|---|---|---|
| PWM   | **P2.1** | TA1.1 | 1–2 ms pulse @ ~50 Hz. **Power servo from a suitable external source**; share GND with MSP430. |

### Ultrasonic Sensor (e.g., HC‑SR04‑like)
| Signal | MSP430 Pin | Notes |
|---|---|---|
| TRIG  | **P2.3** | GPIO output, 10 µs pulse |
| ECHO  | **P2.2** (TA1 CCI1B) | Timer_A1 capture input (use level‑shift/divider if ECHO is 5V) |

### LDR Light Sensors (×2, analog read)
| Sensor | MSP430 Pin | ADC Channel | Typical wiring |
|---|---|---|---|
| LDR1  | **P1.5** | A5 | Voltage divider to 3.3V; midpoint to ADC |
| LDR2  | **P1.7** | A7 | 〃 |

#### Quick pin summary
- UART: P1.1(RX), P1.2(TX)
- LCD (4‑bit): RS=P1.3, RW=P1.4(low), E=P1.0, D4..D7=P2.4..P2.7
- Buttons: PB0=P2.0, PB1=P1.6 (both pull‑up, active‑low)
- Servo PWM: P2.1 (TA1.1)
- Ultrasonic: TRIG=P2.3, ECHO=P2.2 (Timer_A1 capture)
- LDRs: P1.5(A5), P1.7(A7) as divider midpoints
