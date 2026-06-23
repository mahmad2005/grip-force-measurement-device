# GripForce Measurement Device - Project Instructions

Last Updated: June 22, 2026

---

## 1. Project Overview

This is a comprehensive **hand grip force and object trajectory measurement system** designed for clinical and research applications, particularly for **neurological assessment** of patients with **stroke** or **Multiple Sclerosis (MS)**.

The system combines:
- **Pressure sensing** (32×64 grid, 2048 spatial samples)
- **IMU tracking** (Gyroscope, Accelerometer, Magnetometer)
- **Vision-based pose estimation** (AprilTag detection for 4K video)
- **Real-time visualization** (Python visualizers + web-based 3D viewer)
- **Data logging and playback** (NDJSON pressure/IMU sessions, viewer data export)

---

## 2. Main Purpose

**Objective**: Enable clinicians and researchers to:
- Quantitatively assess **hand grip strength and control** during task execution
- Track and visualize **object movement and orientation** in real-time
- Analyze **neuromuscular impairments** and rehabilitation outcomes
- Record and replay synchronized pressure, IMU, and video data

**Use Cases**:
- Stroke recovery evaluation (motor control assessment)
- MS patient monitoring (grip strength trends)
- Hand rehabilitation research
- Biomechanical analysis of gripping tasks

---

## 3. Current App Features

### Pressure Distribution Visualization
- **32×64 heatmap** of grip force across sensor pad
- Real-time animated pressure overlay with configurable colormap (default: rainbow)
- Pressure range: 0–4095 (raw ADC counts)
- Automatic or manual color scaling (VMIN/VMAX)
- Supports **flipped**, **rotated**, and **half-mirror** transformations for sensor orientation

### IMU Data Visualization
- **Gyroscope**: angular velocity (°/s) at ±250 dps range
- **Accelerometer**: linear acceleration (g) at ±2 g range
- **Magnetometer**: magnetic field (µT) AK09916 sensor (0.15 µT/LSB)
- Real-time animated plots with configurable sliding window (default: 800 samples)
- Optional magnitude-only view (single trace per sensor)
- Per-component plots (X, Y, Z channels)

### 3D Visualization Options
- Optional **3D cylinder object** rendered to match object orientation (from IMU rotation matrix)
- Interactive 3D view with matplotlib 3D axes
- Real-time quaternion-to-matrix conversion

### Data Logging
- Optional **NDJSON session recording** (one JSON object per line)
- Logs all pressure frames and IMU data with timestamps
- Supports replay in web viewer for later analysis

### 3D Simulator
- Realistic **moving pressure patterns** (circular blob with velocity)
- Simulated IMU data (smooth gyro/accel/mag oscillations)
- No hardware required for testing and development
- Interactive GUI option available

### Web Viewer & AprilTag Export
- **Standalone desktop viewer** (via pywebview) or browser
- AprilTag-based **4K video pose detection** and annotation
- Exports **viewer_data.json** and **viewer.html** with embedded annotated frame data
- Supports **start/end frame selection** for video processing
- Video resizing for browser optimization (720p, 360p, 240p variants)

---

## 4. Important Files and What Each Does

### Core Visualizer Scripts (`app/device_data_acquisitiom/`)

| File | Purpose |
|------|---------|
| `viz_full_with_gyro_accel_mag.py` | Standard 2D visualizer: pressure heatmap + IMU plots |
| `viz_full_with_gyro_accel_mag_3Dobj.py` | Enhanced 2D visualizer + optional 3D cylinder object |
| `simulator.py` | Simulates realistic pressure/IMU data without hardware |
| `simulator_gui.py` | Interactive GUI for simulator (if available) |
| `run_viz_app.bat` | Batch launcher for standard visualizer |
| `run_viz_3D.bat` | Batch launcher for 3D visualizer |

### Viewer & Pose Estimation (`app/device_pose_acquisition/auto_target_detections/`)

| File | Purpose |
|------|---------|
| `detect_apriltag_pose_fixed11_4k.py` | Primary AprilTag detector for 4K video; exports viewer data |
| `viewer_7371/viewer.html` | Web-based viewer template (portable, self-contained) |
| `viewer_7371/viewer_data.json` | Exported frame data (sanitized numerics) |
| `viewer_7371/standalone_viewer.py` | Desktop launcher for viewer.html via pywebview |
| `run_viewer_standalone.bat` | Batch launcher for desktop viewer |

### Session Data (`app/device_pose_acquisition/4k_video_process/sessions/`)

| File | Purpose |
|------|---------|
| `session1.ndjson`, `session_2.ndjson` | Recorded pressure + IMU frames (NDJSON format) |

### Hardware & Firmware

| File | Purpose |
|------|---------|
| `firmware/esp32/ESP32_C3_Zero_SPI_2048_UDP.ino` | ESP32 firmware: SPI slave, UDP broadcaster |
| `firmware/stm32f103c8t6/Core/Src/main.c` | STM32 firmware: scans 32×64 matrix via ADG732 multiplexers |
| `firmware/stm32f103c8t6/Drivers/...` | STM32 HAL and CMSIS libraries |

### Documentation

| File | Purpose |
|------|---------|
| `docs/System_Protocol_and_Wiring.md` | Pinouts, SPI/UDP frames, endianness, timing |
| `hardware/README.md` | KiCad project, BOM, component references |
| `hardware/pcb design/GripForceMeasurementDeviceCkt_STM32F103C8T6/` | KiCad schematics and board layout |

---

## 5. Data Formats

### 5.1 Pressure Data Format

**Source**: ESP32 reads 2048 uint16 values via SPI from STM32, then broadcasts over UDP

**UDP Frame Structure** (415 bytes per packet):
```
[0]      0xAA               (start marker)
[1]      0x55               (start marker)
[2]      chunk_index        (0–9, indicating position in frame)
[3..412] 410 bytes          (205 × uint16 values, LSB first per value)
[413]    0x55               (end marker)
[414]    0xAA               (end marker)
```

**Per-Frame Assembly**:
- 10 UDP packets (indices 0–9) concatenated in order
- Total: 2050 values; visualizer uses first **2048 values**
- Reshape as **32 (rows) × 64 (columns)** matrix

**Data Type**: `uint16`, little-endian
**Range**: 0–4095 (raw 12-bit ADC)
**Update Rate**: ~60 fps (adjustable via firmware delay)

**Default UDP Endpoint**: `0.0.0.0:12345`

---

### 5.2 IMU Data Format

**Source**: ESP32 reads ICM-20948 sensor (6-pin module) via SPI

**Packet Header**: `b'\xAA\x55IM'` (4 bytes) + data

**Data Structure (22 bytes total)**:
- 9 × `int16` (big-endian) for: Gyro(X,Y,Z) + Accel(X,Y,Z) + Mag(X,Y,Z)

**Scale Factors**:
| Sensor | Range | LSB | Units |
|--------|-------|-----|-------|
| Gyro | ±250 dps | 131.0 | deg/s |
| Accel | ±2 g | 16384.0 | g |
| Mag | ±120 µT | 0.15 | µT |

**Fallback Header** (if mag unavailable): `b'\xAA\x55GY'` (16 bytes, no magnetometer)

---

### 5.3 Viewer Data Format (JSON Export)

**Purpose**: Portable frame and annotation data for web viewer

**Structure**:
```json
{
  "frames": [
    {
      "frame_idx": 0,
      "timestamp_ms": 1234567890,
      "circles": [
        {"x_px": 100, "y_px": 200, "radius_px": 20, "label": "A"}
      ],
      "detections": [
        {"x": 10.5, "y": 20.3, "z": 5.1, "rx": 0.1, "ry": 0.2, "rz": 0.3}
      ]
    }
  ],
  "meta": {"video_fps": 30, "total_frames": 150}
}
```

**Generated By**: `detect_apriltag_pose_fixed11_4k.py --export-viewer-data <path>`

**Outputs**:
- `viewer_data.json` — sanitized numeric data
- `viewer.html` — self-contained HTML with embedded data

---

### 5.4 Session Data Format (NDJSON)

**Purpose**: Streaming-safe pressure + IMU recording for replay

**Format**: One JSON object per line (newline-delimited)

**Metadata Line**:
```json
{
  "type": "meta",
  "schema": "kinesiology.pressure_imu.ndjson.v1",
  "created_unix_s": 1781914344.1745646,
  "frame_shape": [32, 64],
  "pressure_dtype": "uint16",
  "pressure_layout": "row-major-flat",
  "imu_units": {"gyro": "deg_s", "accel": "g", "mag": "uT"}
}
```

**Frame Line** (repeating):
```json
{
  "type": "frame",
  "frame_idx": 0,
  "t_rel_s": 0.8313055038452148,
  "t_unix_s": 1781914345.0028675,
  "pressure_u16_flat": [0, 0, 6, 25, 11, 5, ...],
  "gyro_deg_s": [50.0, 30.0, 20.0],
  "accel_g": [0.1, 0.05, 1.0],
  "mag_uT": [40.0, 40.0, 30.0]
}
```

**Used By**: Web viewer for replay, analysis tools

---

### 5.5 Video & Annotation Formats

**Input**: 4K MP4 (e.g., `IMG_0005.MOV`, typically 60 fps)

**Processing**: AprilTag detection extracts 3D pose per frame

**Output**: 
- Annotated MP4 with overlaid circles/detections
- Resized variants (720p, 360p, 240p) for web playback
- Viewer HTML with frame-by-frame data

**FFmpeg Resize Commands** (stored in Note.txt):
```bash
# 720p
ffmpeg -y -i input.mp4 -vf "scale=1280:-2" -c:v libx264 ... output_720p.mp4

# 360p, 15fps
ffmpeg -y -i input.mp4 -vf "fps=15,scale=-2:360" -c:v libx264 ... output_360p_15fps.mp4
```

---

## 6. Important Rules for Future Editing

### 6.1 Do Not Delete or Simplify Features

- **Pressure distribution sensor ratio must stay fixed at 32 × 64**
- **Do not remove IMU support** (gyro, accel, mag)
- **Preserve older project functionality** when fixing bugs
- Do not simplify AprilTag viewer workflow
- Keep all batch launcher files (`.bat`) for user convenience

### 6.2 Heatmap & UI Constraints

- **Heatmap should resize correctly without stretching or squeezing**
  - Maintain aspect ratio; use proper matplotlib grid layout
- **Pressure distribution panel should show heatmap and controls without unnecessary scrolling**
  - Use GridSpec to balance plot sizes
- **Color mapping should be configurable** without code changes (VMIN, VMAX, colormap)

### 6.3 Data Synchronization

- **Video, pressure data, and viewer/pose data should stay synchronized**
  - Use Unix timestamps (`t_unix_s`) for alignment
  - Maintain frame index (`frame_idx`) continuity
- **Bug fixes should be minimal and should not break other panels**
  - Test all visualizer panels after edits
  - Preserve UDP packet format (10 packets per frame, 415 bytes each)

### 6.4 Configuration & Defaults

- **Default UDP endpoint**: `0.0.0.0:12345`
- **Default simulator IP/port**: `127.0.0.1:12345`
- **Default frame size**: 32 rows × 64 columns
- **Default pressure range**: 0–4095
- **Default IMU scales**: Gyro 131.0, Accel 16384.0, Mag 0.15
- **Do not change these without updating documentation and all dependent scripts**

### 6.5 Version Control

- Keep `.gitignore` as-is (ignores `*.mp4`, `*.pyc`, `__pycache__/`, etc.)
- Commit changes with descriptive messages
- Tag releases with version numbers

---

## 7. Known Issues

### Needs Confirmation (Not Fully Tested)

| Issue | Impact | Status |
|-------|--------|--------|
| Video resolution auto-scaling in AprilTag detector | Low | Unconfirmed |
| Magnetometer availability fallback (`GY` vs `IM` headers) | Low | Unconfirmed |
| Multi-monitor layout for visualizer | Low | Unconfirmed |
| Very large session files (>1GB NDJSON) | Medium | Unconfirmed |

### Potential Improvements (No Bugs Reported)

- Web viewer performance with large datasets (>10,000 frames)
- Pressure sensor drift calibration over long sessions
- Bluetooth stability on Windows 10/11

---

## 8. Development / Run Instructions

### 8.1 Quick Start (Windows CMD)

**Simulator (No Hardware)**:
```bash
cd app\device_data_acquisitiom
python simulator.py --ip 127.0.0.1 --port 12345 --fps 60
```

**Start Visualizer** (in separate terminal):
```bash
cd app\device_data_acquisitiom
python viz_full_with_gyro_accel_mag.py --ip 0.0.0.0 --port 12345 --show-magnitude
```

Or use batch files:
```bash
run_viz_app.bat       # Standard visualizer
run_viz_3D.bat        # 3D visualizer with cylinder
run_viewer_standalone.bat  # Web viewer (desktop)
```

### 8.2 With Hardware (ESP32 + STM32 + Sensor Pad)

1. **Program ESP32** with `firmware/esp32/ESP32_C3_Zero_SPI_2048_UDP.ino`
   - Set `udpAddress` to your PC's IP
   - Set `udpPort = 12345` (or custom)
   - Configure WiFi SSID/password

2. **Program STM32F103C8T6** with firmware from `firmware/stm32f103c8t6/`
   - Use STM32CubeProgrammer
   - Or compile in STM32CubeIDE and program via JTAG

3. **Connect sensor pad** to STM32 ADG732 multiplexer chain
   - Verify pinouts in `docs/System_Protocol_and_Wiring.md`

4. **Start visualizer**:
   ```bash
   python viz_full_with_gyro_accel_mag.py --ip 0.0.0.0 --port 12345 --show-magnitude
   ```

### 8.3 AprilTag Video Processing

**Basic Usage**:
```bash
cd app\device_pose_acquisition\auto_target_detections
python detect_apriltag_pose_fixed11_4k.py --video input.mp4 --annotated-out output.mp4
```

**With Viewer Export**:
```bash
python detect_apriltag_pose_fixed11_4k.py \
  --video input.mp4 \
  --annotated-out output.mp4 \
  --export-viewer-data viewer_folder \
  --process-fps 15 \
  --max-frames 0
```

**Optional Parameters**:
- `--start-frame N` — start processing at frame N
- `--end-frame N` — stop at frame N
- `--undistort` — apply camera distortion correction
- `--max-frames N` — process max N frames (0 = all)

### 8.4 Install Python Dependencies

```bash
python -m pip install -r requirements.txt
```

**Key packages**:
- `numpy`, `scipy`
- `matplotlib` (for 2D/3D plotting)
- `opencv-python` (for video/image processing)
- `pupil-apriltags` (for AprilTag detection)
- `pywebview` (for standalone viewer)

---

## 9. Future Work

### Short-term (Next 1–3 Months)

- [ ] **Calibration UI**: Interactive pressure sensor calibration wizard
- [ ] **Real-time statistics panel**: Live grip strength metrics (max, mean, duration)
- [ ] **Patient profile system**: Save/load patient settings and historical data
- [ ] **Session playback controls**: Play, pause, slow-motion for recorded NDJSON

### Medium-term (3–6 Months)

- [ ] **Cloud storage integration**: Upload sessions to cloud for remote analysis
- [ ] **Batch video processing**: Queue multiple videos for AprilTag detection
- [ ] **Advanced analytics dashboard**: Trend analysis, comparison across sessions
- [ ] **Mobile app MVP**: Lightweight grip strength monitoring on Android/iOS

### Long-term (6+ Months)

- [ ] **AI-based grip classification**: ML model for detecting grip patterns (pinch, power, etc.)
- [ ] **Multi-patient database**: Centralized data management with role-based access
- [ ] **Integration with EHR systems**: Export data to hospital databases
- [ ] **Predictive rehabilitation model**: ML-based recovery outcome prediction

---

## 10. Needs Confirmation

### Questions for the User

1. **Screenshot / Screenshots**:
   - A new screenshot `new_gripforce_3dsimulator_outlook.png` exists in `app/`. Should it be featured prominently in README?
   - Are there other screenshots or demo videos you'd like to add?

2. **Demonstration Video**:
   - A new video URL was provided: `https://www.youtube.com/watch?v=0S4J9DHq-p0`
   - What is this video about? (GripForce 3D viewer demo? New features? User workflow?)
   - Should it be the primary video, or kept alongside the existing AprilTag video?

3. **Web Viewer Path**:
   - Is `web_viz_app/` a typo in app/README.md, or does this folder exist elsewhere?
   - Should references be updated to `device_pose_acquisition/auto_target_detections/viewer_7371/`?

4. **Session Data Location**:
   - Sessions are stored in `app/device_pose_acquisition/4k_video_process/sessions/`
   - Should users be able to upload/download sessions? Is cloud storage planned?

5. **Calibration & Sensor Drift**:
   - Is pressure sensor calibration a known issue? Should it be documented?
   - Are there any reference measurements or known sensor limitations?

6. **Performance Targets**:
   - What is the target frame rate? (Currently ~60 fps, documented as adjustable)
   - What is the maximum session duration before performance degrades?

7. **Hardware Variants**:
   - Are there other MCU variants beyond STM32F103C8T6 and ESP32-C3?
   - Should this project support modular hardware swapping?

8. **Export Formats**:
   - Should CSV export be supported (in addition to NDJSON)?
   - What are the preferred formats for clinical reports?

---

## Revision History

| Date | Version | Changes |
|------|---------|---------|
| 2026-06-22 | 1.0 | Initial comprehensive documentation |

---

