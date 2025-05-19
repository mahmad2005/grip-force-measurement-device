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



## 📂 Previous Design

This section showcases the earlier versions of the PCB and sensor module designs for the Hand Grip Force Measurement Device.

| 3D View | 3D View (with Bluetooth Module) |
|--------|-------------------------------|
| ![3D View](hardware/previous%20design/images/3dview.png) | ![3D View BTH Module](hardware/previous%20design/images/3dview_bth_module.png) |

| PCB Layout | PCB Layout (with Bluetooth Module) |
|------------|------------------------------------|
| ![PCB](hardware/previous%20design/images/pcb.png) | ![PCB BTH Module](hardware/previous%20design/images/pcb_bth_module.png) |



## 🔧 New Design

This section features the redesigned pressure sensor and control PCB with updated layout and schematic.

| Front View | Back View |
|------------|------------|
| ![3D View Front](hardware/pcb%20design/images/3dviewfront.png) | ![3D View Back](hardware/pcb%20design/images/3dviewback.png) |

### 🧩 Schematic

![Schematic](hardware/pcb%20design/images/schemetic.png)

### 3D View of the Pressure Sensor Pad
![Pressure Sensor Pad 3D](/hardware/sensors%20design/pressure%20sensor%20pad%20design/pressureSensorPadFootPrint_v2_3D.png)