# Hardware overview

This folder contains the KiCad project, 3D models, and prior design assets for the Grip Force Measurement Device.

## KiCad project

Project: `pcb design/GripForceMeasurementDeviceCkt_STM32F103C8T6/`

Key files:
- `GripForceMeasurementDeviceCkt_STM32F103C8T6.kicad_sch` — schematic
- `GripForceMeasurementDeviceCkt_STM32F103C8T6.kicad_pcb` — board layout
- `GripForceMeasurementDeviceCkt_STM32F103C8T6.pdf` — exported drawing/PDF (present)
- `gerberAll/` — manufacturing outputs

3D models and references: see `pcb design/3D-Models-Step-files/`

## BOM highlights (from IPC-2581 XML)

- MCU: STM32F103C8Tx (LQFP‑48)
- Multiplexers: ADG732BSUZ (16‑to‑1 analog switch, multiple devices used to reach 32×64)
- Wireless module: ESP32‑C3‑Mini (through‑hole variant footprint)
- IMU header: SparkFun ICM‑20948 module (6‑pin socket)

For a full BOM, open the KiCad project or review the IPC‑2581 XML at:
`pcb design/GripForceMeasurementDeviceCkt_STM32F103C8T6/GripForceMeasurementDeviceCkt_STM32F103C8T6.xml`

## Notes

- Signal levels are 3V3.
- SPI runs between ESP32‑C3 (master) and STM32F103 (slave). Ensure a clean ground reference and short interconnects.
- The sensor matrix connects to the ADG732 network; ADC measures on PA0.
