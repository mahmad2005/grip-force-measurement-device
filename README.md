# Hand Grip Force and Trajectory Measurement System

## Overview

This project focuses on the development of a portable, intelligent device designed to measure **hand grip force** and **object trajectory** for neurological assessment, particularly in patients with **stroke** or **Multiple Sclerosis (MS)**.

## Key Features

- 🧠 **Neurological Focus**  
  Designed to evaluate motor function, coordination, and rehabilitation progress in stroke and MS patients.

- 🔧 **Core Technologies**
  - **STM32 Microcontrollers** – Real-time data acquisition and processing
  - **Pressure Sensors** – High-resolution spatial grip force measurement
  - **Gyroscope & IMU Sensors** – Motion and orientation tracking of held objects
  - **Bluetooth** – Wireless data transmission to PC/mobile apps
  - **RTOS** – Deterministic multitasking on embedded systems
  - **Embedded Linux** – Advanced data handling and UI support

## System Architecture

[ Pressure Sensor ] [ Gyroscope / IMU ]
| |
[ Analog Front End ] |
| |
[ STM32 MCU ] <--> [ Bluetooth Module ]
|
[ Data Logger / Wi-Fi ]
|
[ Embedded Linux Platform ]


## Use Case

The system enables clinicians and researchers to:
- Quantitatively assess hand grip strength and control
- Track and visualize object movement during gripping tasks
- Analyze neuromuscular impairments and rehabilitation outcomes

## Future Work

- 🔬 Integration with AI models for predictive analysis  
- 📱 Mobile app for real-time data visualization  
- 🌐 Cloud-based data synchronization and reporting

---

**Developed using:**  
`C/C++ (STM32 HAL)`, `FreeRTOS`, `Python`, `Embedded Linux`, `Bluetooth Low Energy (BLE)`

---

> 📍 *This project is part of ongoing research in neuromuscular assessment and assistive rehabilitation technology.*
