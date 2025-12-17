# STM32 UART Command Interface
## Overview

This project implements a from-scratch UART interface and command-processing system on an STM32 microcontroller. The goal is to build a functional serial command interface using direct register-level programming, without relying on vendor HAL drivers for UART functionality.

The project focuses on understanding how UART hardware works at a low level and how higher-level abstractions (read/write, buffering, command parsing) are constructed on top of it in real embedded systems.
