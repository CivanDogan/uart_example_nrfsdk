# UART LED Control Project for nRF54L15

This project demonstrates how to control LEDs using UART input on a Zephyr-based system, specifically for the nRF54L15 board. The application initializes UART and GPIO devices, listens for UART input, and toggles LEDs based on received commands.

## Features

- UART communication for receiving commands.
- Control up to 4 LEDs using UART input.
- Initial message sent over UART to guide the user.
- LED blinking to indicate initialization completion.

## Hardware Requirements

- nRF54L15 board with UART and GPIO capabilities.
- LEDs connected to GPIO pins defined in the device tree.

## Software Requirements

- Zephyr RTOS
- Zephyr SDK

## Setup

1. **Clone the repository:**
    ```sh
    git clone <repository_url>
    cd <repository_directory>
    ```

2. **Build the project:**
    ```sh
    west build -b nrf54l15 -s .
    ```

3. **Flash the firmware:**
    ```sh
    west flash
    ```

## Usage

1. **Connect to the UART interface** using a serial terminal (e.g., `minicom`, `screen`, `putty`).
2. **Send commands**:
    - `1` to toggle LED1
    - `2` to toggle LED2
    - `3` to toggle LED3
    - `4` to toggle LED4

## Code Overview

- **main.c**: Contains the main application logic.
  - Initializes UART and LEDs.
  - Defines UART receive callback to process incoming data.
  - Toggles LEDs based on received UART commands.

## Author

Civan
