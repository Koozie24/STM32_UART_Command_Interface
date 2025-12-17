# STM32 UART Command Interface
## Overview

This project implements a from-scratch UART interface and command-processing system on an STM32 microcontroller. The goal is to build a functional serial command interface using direct register-level programming, without relying on vendor HAL drivers for UART functionality.

The project focuses on understanding how UART hardware works at a low level and how higher-level abstractions (read/write, buffering, command parsing) are constructed on top of it in real embedded systems.

### <ins>Platform:</ins> 
#### *STM32 NUCLEO-F401RE*

## Project Goals:
### No HAL or LL Libraries
### 1. Implement UART Driver
- Configure and manipulate registers for GPIOA/USART2 pins.
- Implement custom TX/RX routine.
### 2. Custom I/O for UART
- Overwrite syscall _read and _write to develop functionality to send and receive data through putty terminal.
- Develop echo and buffer for CLI
- Interrupt handling
- Capture input from terminal and send to over UART
### 3. Do Something with Input
- Command to toggle on/off led
- Set led blink speed
- Set led blink pattern
- Something else - more leds/some other action/control
