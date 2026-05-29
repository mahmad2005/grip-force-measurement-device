#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simulator_gui.py — Full GUI Virtual Hardware Testing Environment
for the 32×64 Grip-Pressure + IMU Cylinder System.

Architecture:
  SimulatorCore   — shared state / logic (sensor ranges, presets, recording)
  IMUModel        — generates / holds IMU values (auto or manual)
  PressureModel   — generates / holds pressure matrix (auto or manual)
  UDPStreamer     — background thread that encodes and sends UDP packets
  GUIController   — tkinter window, sliders, preview canvas, stats

Packet format is identical to the real device — existing visualizer works
without modification.

Run:
    python simulator_gui.py --ip 127.0.0.1 --port 12345
"""

import argparse
import csv
import math
import os
import socket
import struct
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# PROTOCOL CONSTANTS  (must match viz_full_with_gyro_accel_mag_3Dobj.py)
# ─────────────────────────────────────────────────────────────────────────────
FRAME_W, FRAME_H = 64, 32
FRAME_SIZE = FRAME_W * FRAME_H
CHUNKS_PER_FRAME = 10
VALUES_PER_CHUNK = 205          # 205 uint16 values per pressure chunk
BYTES_PER_PACKET = 415          # 2 + 1 + 410 + 2 = 415

HDR_IM = b"\xAA\x55IM"         # IMU full packet header (gyro+accel+mag)

# ── Sensor scaling  (modify here to match your hardware)
GYRO_SF    = 131.0              # raw LSB per deg/s   (±250 dps mode)
ACC_SF     = 16384.0            # raw LSB per g        (±2 g mode)
MAG_SF_uT  = 0.15               # µT per raw LSB       (AK09916)

# ── Physical sensor limits  (edit to change slider range)
GYRO_RANGE = 250.0              # ±250 deg/s
ACC_RANGE  = 2.0                # ±2 g
MAG_RANGE  = 120.0              # ±120 µT

PRESSURE_MIN = 0
PRESSURE_MAX = 4095

# ─────────────────────────────────────────────────────────────────────────────
# PRESETS
# To add a new preset: add an entry to PRESETS dict.
# Each preset is a callable that receives elapsed time t and returns
# (gx, gy, gz, ax, ay, az, mx, my, mz, cx, cy, radius, strength).
# ─────────────────────────────────────────────────────────────────────────────
def _preset_gripping(t):
    return (
        2*math.sin(t*0.3), 1*math.cos(t*0.2), 0.5*math.sin(t*0.1),
        0.0, 0.0, 1.0,
        40*math.cos(t*0.05), 40*math.sin(t*0.05), 30.0,
        FRAME_W//2, FRAME_H//2, 10, 3800,
    )

def _preset_rolling(t):
    angle = t * 1.2
    return (
        0.0, 80*math.cos(angle), 0.0,
        math.sin(angle), 0.0, math.cos(angle),
        40*math.cos(t*0.1), 40*math.sin(t*0.1), 30.0,
        int(FRAME_W//2 + (FRAME_W//3)*math.cos(angle)),
        int(FRAME_H//2 + (FRAME_H//4)*math.sin(angle*0.7)),
        8, 3200,
    )

def _preset_shaking(t):
    s = math.sin(t * 8.0)
    return (
        200*s, 150*math.cos(t*9), 100*math.sin(t*7),
        0.8*s, 0.6*math.cos(t*9), 1.0+0.3*s,
        40.0, 20*s, 30.0,
        int(FRAME_W//2 + 5*s),
        int(FRAME_H//2 + 3*math.cos(t*9)),
        6, 3000,
    )

def _preset_twisting(t):
    gz = 120 * math.sin(t * 0.8)
    return (
        5*math.sin(t*0.2), 5*math.cos(t*0.2), gz,
        0.1*math.sin(t*0.4), 0.1*math.cos(t*0.4), 0.99,
        40*math.cos(t*0.3), 40*math.sin(t*0.3), 30.0,
        int(FRAME_W//2 + (FRAME_W//3)*math.cos(t*0.5)),
        FRAME_H//2, 9, 3500,
    )

def _preset_static(t):
    return (
        0.0, 0.0, 0.0,
        0.0, 0.0, 1.0,
        40.0, 0.0, 30.0,
        FRAME_W//2, FRAME_H//2, 10, 3600,
    )

def _preset_random(t):
    rng = np.random.default_rng(int(t*10) & 0xFFFF)
    return (
        float(rng.uniform(-150, 150)),
        float(rng.uniform(-150, 150)),
        float(rng.uniform(-150, 150)),
        float(rng.uniform(-1.5, 1.5)),
        float(rng.uniform(-1.5, 1.5)),
        float(rng.uniform(0.5, 1.5)),
        float(rng.uniform(-80, 80)),
        float(rng.uniform(-80, 80)),
        float(rng.uniform(-60, 60)),
        int(rng.integers(5, FRAME_W-5)),
        int(rng.integers(5, FRAME_H-5)),
        int(rng.integers(4, 14)),
        int(rng.integers(2000, 4095)),
    )

PRESETS = {
    "Gripping":      _preset_gripping,
    "Rolling":       _preset_rolling,
    "Shaking":       _preset_shaking,
    "Twisting":      _preset_twisting,
    "Static Hold":   _preset_static,
    "Random Motion": _preset_random,
}


# ─────────────────────────────────────────────────────────────────────────────
# HELPER
# ─────────────────────────────────────────────────────────────────────────────
def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ─────────────────────────────────────────────────────────────────────────────
# PRESSURE MODEL
# Generates the 32×64 pressure matrix.
# In AUTO mode, a moving blob follows a Lissajous path.
# In MANUAL mode, values come directly from the shared state.
# ─────────────────────────────────────────────────────────────────────────────
class PressureModel:
    def generate(self, cx: float, cy: float, radius: float, strength: float) -> np.ndarray:
        y_idx, x_idx = np.ogrid[:FRAME_H, :FRAME_W]
        mask = (x_idx - cx)**2 + (y_idx - cy)**2 <= radius**2
        arr = np.zeros((FRAME_H, FRAME_W), dtype=np.uint16)
        arr[mask] = int(strength)
        # Add small random noise to make it look realistic
        arr = arr + np.random.randint(0, 60, arr.shape, dtype=np.uint16)
        arr = np.clip(arr, PRESSURE_MIN, PRESSURE_MAX)
        return arr

    @staticmethod
    def auto_position(t: float):
        """Returns (cx, cy) for automatic Lissajous movement."""
        cx = FRAME_W // 2 + int((FRAME_W // 3) * math.cos(t * 0.5))
        cy = FRAME_H // 2 + int((FRAME_H // 3) * math.sin(t * 0.7))
        return cx, cy


# ─────────────────────────────────────────────────────────────────────────────
# IMU MODEL
# Holds current IMU readings. In AUTO mode, returns sinusoidal values.
# In MANUAL mode, returns whatever the shared state contains.
# ─────────────────────────────────────────────────────────────────────────────
class IMUModel:
    @staticmethod
    def auto_values(t: float):
        """Smooth sinusoidal IMU simulation."""
        gx = 50 * math.sin(t * 0.7)
        gy = 30 * math.cos(t * 0.5)
        gz = 20 * math.sin(t * 0.3)
        ax = math.sin(t * 0.2)
        ay = math.cos(t * 0.2)
        az = 1.0 + 0.05 * math.sin(t * 0.1)
        mx = 40 * math.cos(t * 0.1)
        my = 40 * math.sin(t * 0.1)
        mz = 30 * math.cos(t * 0.05)
        return gx, gy, gz, ax, ay, az, mx, my, mz


# ─────────────────────────────────────────────────────────────────────────────
# UDP STREAMER
# Background thread that reads from SimulatorCore and sends packets.
# ─────────────────────────────────────────────────────────────────────────────
class UDPStreamer(threading.Thread):
    def __init__(self, core):
        super().__init__(daemon=True)
        self._core = core
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def run(self):
        core = self._core
        while not core.quit_flag:
            if not core.streaming:
                time.sleep(0.05)
                continue

            t_loop_start = time.perf_counter()
            t = time.time() - core.t0

            # ── Determine IMU values
            if core.preset_fn is not None:
                vals = core.preset_fn(t)
                gx, gy, gz = vals[0], vals[1], vals[2]
                ax, ay, az = vals[3], vals[4], vals[5]
                mx, my, mz = vals[6], vals[7], vals[8]
                cx, cy    = vals[9], vals[10]
                rad       = vals[11]
                strength  = vals[12]
                core.state["cx"] = cx
                core.state["cy"] = cy
                core.state["radius"] = rad
                core.state["strength"] = strength
                core.state["gx"] = gx; core.state["gy"] = gy; core.state["gz"] = gz
                core.state["ax"] = ax; core.state["ay"] = ay; core.state["az"] = az
                core.state["mx"] = mx; core.state["my"] = my; core.state["mz"] = mz
            elif core.state["imu_manual"]:
                gx, gy, gz = core.state["gx"], core.state["gy"], core.state["gz"]
                ax, ay, az = core.state["ax"], core.state["ay"], core.state["az"]
                mx, my, mz = core.state["mx"], core.state["my"], core.state["mz"]
            else:
                gx, gy, gz, ax, ay, az, mx, my, mz = IMUModel.auto_values(t)
                core.state["gx"] = gx; core.state["gy"] = gy; core.state["gz"] = gz
                core.state["ax"] = ax; core.state["ay"] = ay; core.state["az"] = az
                core.state["mx"] = mx; core.state["my"] = my; core.state["mz"] = mz

            # ── Determine pressure position
            if core.preset_fn is None and not core.state["pressure_manual"]:
                cx, cy = PressureModel.auto_position(t)
                core.state["cx"] = cx
                core.state["cy"] = cy

            cx       = core.state["cx"]
            cy       = core.state["cy"]
            radius   = core.state["radius"]
            strength = core.state["strength"]

            # ── Build pressure matrix
            mat = core.pressure_model.generate(cx, cy, radius, strength)
            core.last_mat = mat  # expose for GUI preview

            # ── Pack and send pressure packets
            dest = (core.ip, core.port)
            flat = mat.flatten()
            for i in range(CHUNKS_PER_FRAME):
                chunk = flat[i * VALUES_PER_CHUNK:(i + 1) * VALUES_PER_CHUNK]
                payload = chunk.astype("<u2").tobytes()
                if len(payload) < 410:
                    payload += b"\x00" * (410 - len(payload))
                pkt = bytearray(b"\xAA\x55")
                pkt.append(i)
                pkt += payload
                pkt += b"\x55\xAA"
                self._sock.sendto(bytes(pkt), dest)

            # ── Pack and send IMU packet
            def _i16(v):
                return int(_clamp(v, -32768, 32767))

            imu_vals = [
                _i16(gx * GYRO_SF), _i16(gy * GYRO_SF), _i16(gz * GYRO_SF),
                _i16(ax * ACC_SF),  _i16(ay * ACC_SF),  _i16(az * ACC_SF),
                _i16(mx / MAG_SF_uT), _i16(my / MAG_SF_uT), _i16(mz / MAG_SF_uT),
            ]
            imu_pkt = struct.pack(">4s9h", HDR_IM, *imu_vals)
            self._sock.sendto(imu_pkt, dest)

            # ── Recording
            if core.recording and core.csv_writer:
                core.csv_writer.writerow([
                    f"{t:.6f}",
                    f"{gx:.4f}", f"{gy:.4f}", f"{gz:.4f}",
                    f"{ax:.4f}", f"{ay:.4f}", f"{az:.4f}",
                    f"{mx:.4f}", f"{my:.4f}", f"{mz:.4f}",
                    cx, cy, radius, int(strength),
                ])

            # ── Stats
            core.frame_count += 1
            elapsed = time.perf_counter() - t_loop_start
            sleep_t = max(0.0, 1.0 / core.fps - elapsed)
            time.sleep(sleep_t)

        self._sock.close()


# ─────────────────────────────────────────────────────────────────────────────
# SIMULATOR CORE
# Central shared state accessed by both UDPStreamer and GUIController.
# ─────────────────────────────────────────────────────────────────────────────
class SimulatorCore:
    def __init__(self, ip: str, port: int, fps: float):
        self.ip   = ip
        self.port = port
        self.fps  = fps

        self.streaming    = False
        self.quit_flag    = False
        self.recording    = False
        self.csv_fp       = None
        self.csv_writer   = None
        self.replay_rows  = []
        self.preset_fn    = None            # None = use state; callable = preset

        self.t0           = time.time()
        self.frame_count  = 0
        self.last_mat     = np.zeros((FRAME_H, FRAME_W), dtype=np.uint16)

        self.pressure_model = PressureModel()

        # Shared state dict — written by GUI sliders or UDPStreamer (presets/auto)
        self.state = {
            "imu_manual":      False,
            "pressure_manual": False,
            "cx":     float(FRAME_W // 2),
            "cy":     float(FRAME_H // 2),
            "radius": 7.0,
            "strength": 3500.0,
            "gx": 0.0, "gy": 0.0, "gz": 0.0,
            "ax": 0.0, "ay": 0.0, "az": 1.0,
            "mx": 40.0, "my": 0.0, "mz": 30.0,
        }

    def reset_imu(self):
        self.state.update({
            "gx": 0.0, "gy": 0.0, "gz": 0.0,
            "ax": 0.0, "ay": 0.0, "az": 1.0,
            "mx": 40.0, "my": 0.0, "mz": 30.0,
        })

    def start_recording(self, path: str):
        self.csv_fp = open(path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_fp)
        self.csv_writer.writerow([
            "time",
            "gx_dps","gy_dps","gz_dps",
            "ax_g","ay_g","az_g",
            "mx_uT","my_uT","mz_uT",
            "cx","cy","radius","strength",
        ])
        self.recording = True

    def stop_recording(self):
        self.recording = False
        if self.csv_fp:
            self.csv_fp.close()
            self.csv_fp = None
            self.csv_writer = None

    def load_replay(self, path: str) -> int:
        rows = []
        with open(path, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append(row)
        self.replay_rows = rows
        return len(rows)


# ─────────────────────────────────────────────────────────────────────────────
# DARK THEME COLOURS
# ─────────────────────────────────────────────────────────────────────────────
BG      = "#1e1e2e"
BG2     = "#2a2a3e"
BG3     = "#313145"
FG      = "#cdd6f4"
FG2     = "#a6adc8"
ACCENT  = "#89b4fa"
GREEN   = "#a6e3a1"
RED     = "#f38ba8"
YELLOW  = "#f9e2af"
ORANGE  = "#fab387"
MAUVE   = "#cba6f7"

SLIDER_TROUGH = "#45475a"
SLIDER_ACTIVE = "#89b4fa"


# ─────────────────────────────────────────────────────────────────────────────
# GUI CONTROLLER
# ─────────────────────────────────────────────────────────────────────────────
class GUIController:
    PREVIEW_SCALE = 4   # Each pressure cell = 4×4 pixels in preview canvas

    def __init__(self, root: tk.Tk, core: SimulatorCore):
        self.root = root
        self.core = core
        self._fps_last_count = 0
        self._fps_last_time  = time.time()
        self._actual_fps     = 0.0
        self._replay_idx     = 0
        self._replay_active  = False

        self._build_ui()
        self._refresh()   # start periodic GUI refresh loop

    # ────────────────────────────────────────── UI BUILD ─────────────────────
    def _build_ui(self):
        root = self.root
        root.title("Grip-Pressure + IMU Simulator  |  Virtual Hardware")
        root.configure(bg=BG)
        root.resizable(True, True)

        # ── Top bar ──────────────────────────────────────────────────────────
        top = tk.Frame(root, bg=BG, pady=6)
        top.pack(fill="x", padx=8)

        self._btn_start = tk.Button(
            top, text="▶  START STREAM", width=16,
            bg=GREEN, fg="#11111b", font=("Consolas", 10, "bold"),
            relief="flat", command=self._toggle_stream,
        )
        self._btn_start.pack(side="left", padx=4)

        tk.Button(
            top, text="↺  RESET IMU", width=14,
            bg=BG3, fg=FG, font=("Consolas", 10),
            relief="flat", command=self._reset_imu,
        ).pack(side="left", padx=4)

        self._btn_record = tk.Button(
            top, text="⏺  RECORD", width=12,
            bg=BG3, fg=FG, font=("Consolas", 10),
            relief="flat", command=self._toggle_record,
        )
        self._btn_record.pack(side="left", padx=4)

        tk.Button(
            top, text="▷  REPLAY", width=12,
            bg=BG3, fg=FG, font=("Consolas", 10),
            relief="flat", command=self._start_replay,
        ).pack(side="left", padx=4)

        tk.Button(
            top, text="⏹  STOP REPLAY", width=14,
            bg=BG3, fg=FG, font=("Consolas", 10),
            relief="flat", command=self._stop_replay,
        ).pack(side="left", padx=4)

        # Stats label
        self._lbl_stats = tk.Label(
            top, text="", bg=BG, fg=ACCENT,
            font=("Consolas", 9),
        )
        self._lbl_stats.pack(side="right", padx=8)

        # ── Main body — notebook ──────────────────────────────────────────────
        style = ttk.Style()
        style.theme_use("default")
        style.configure("Dark.TNotebook",
                        background=BG, borderwidth=0)
        style.configure("Dark.TNotebook.Tab",
                        background=BG2, foreground=FG,
                        padding=[12, 4], font=("Consolas", 9))
        style.map("Dark.TNotebook.Tab",
                  background=[("selected", BG3)],
                  foreground=[("selected", ACCENT)])

        nb = ttk.Notebook(root, style="Dark.TNotebook")
        nb.pack(fill="both", expand=True, padx=8, pady=4)

        # Tab 1 — sliders
        tab_sliders = tk.Frame(nb, bg=BG)
        nb.add(tab_sliders, text="  Controls  ")

        # Tab 2 — presets
        tab_presets = tk.Frame(nb, bg=BG)
        nb.add(tab_presets, text="  Presets  ")

        # Tab 3 — preview
        tab_preview = tk.Frame(nb, bg=BG)
        nb.add(tab_preview, text="  Live Preview  ")

        self._build_controls(tab_sliders)
        self._build_presets(tab_presets)
        self._build_preview(tab_preview)

    # ── Controls tab ─────────────────────────────────────────────────────────
    def _build_controls(self, parent):
        outer = tk.Frame(parent, bg=BG)
        outer.pack(fill="both", expand=True)

        canvas = tk.Canvas(outer, bg=BG, highlightthickness=0)
        scroll = tk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        inner = tk.Frame(canvas, bg=BG)
        win_id = canvas.create_window((0, 0), window=inner, anchor="nw")

        def _on_configure(e):
            canvas.configure(scrollregion=canvas.bbox("all"))
            canvas.itemconfig(win_id, width=canvas.winfo_width())

        inner.bind("<Configure>", _on_configure)
        canvas.bind("<Configure>", lambda e: canvas.itemconfig(win_id, width=e.width))
        canvas.bind_all("<MouseWheel>",
                        lambda e: canvas.yview_scroll(int(-1*(e.delta/120)), "units"))

        self._sliders = {}
        self._sv      = {}  # StringVar for numeric labels

        # Mode toggles row
        mode_row = tk.Frame(inner, bg=BG2, pady=6, padx=8)
        mode_row.pack(fill="x", padx=8, pady=(8, 2))

        tk.Label(mode_row, text="IMU Mode:", bg=BG2, fg=FG2,
                 font=("Consolas", 9)).pack(side="left", padx=4)
        self._imu_mode_var = tk.StringVar(value="AUTO")
        for mode in ("AUTO", "MANUAL"):
            tk.Radiobutton(
                mode_row, text=mode, variable=self._imu_mode_var, value=mode,
                bg=BG2, fg=FG, selectcolor=BG3, activebackground=BG2,
                font=("Consolas", 9),
                command=self._on_imu_mode_change,
            ).pack(side="left", padx=4)

        tk.Label(mode_row, text="   Pressure Mode:", bg=BG2, fg=FG2,
                 font=("Consolas", 9)).pack(side="left", padx=4)
        self._press_mode_var = tk.StringVar(value="AUTO")
        for mode in ("AUTO", "MANUAL"):
            tk.Radiobutton(
                mode_row, text=mode, variable=self._press_mode_var, value=mode,
                bg=BG2, fg=FG, selectcolor=BG3, activebackground=BG2,
                font=("Consolas", 9),
                command=self._on_pressure_mode_change,
            ).pack(side="left", padx=4)

        # Groups of sliders
        SLIDER_GROUPS = [
            ("GYROSCOPE  (deg/s)", MAUVE, [
                ("gx", -GYRO_RANGE, GYRO_RANGE, 0.0,    "Gyro X"),
                ("gy", -GYRO_RANGE, GYRO_RANGE, 0.0,    "Gyro Y"),
                ("gz", -GYRO_RANGE, GYRO_RANGE, 0.0,    "Gyro Z"),
            ]),
            ("ACCELEROMETER  (g)", ACCENT, [
                ("ax", -ACC_RANGE, ACC_RANGE, 0.0,  "Accel X"),
                ("ay", -ACC_RANGE, ACC_RANGE, 0.0,  "Accel Y"),
                ("az", -ACC_RANGE, ACC_RANGE, 1.0,  "Accel Z"),
            ]),
            ("MAGNETOMETER  (µT)", YELLOW, [
                ("mx", -MAG_RANGE, MAG_RANGE, 40.0, "Mag X"),
                ("my", -MAG_RANGE, MAG_RANGE,  0.0, "Mag Y"),
                ("mz", -MAG_RANGE, MAG_RANGE, 30.0, "Mag Z"),
            ]),
            ("PRESSURE", GREEN, [
                ("cx",       0, FRAME_W-1, float(FRAME_W//2), "Centre X"),
                ("cy",       0, FRAME_H-1, float(FRAME_H//2), "Centre Y"),
                ("radius",   1, 20,        7.0,               "Radius"),
                ("strength", PRESSURE_MIN, PRESSURE_MAX, 3500.0, "Strength"),
            ]),
            ("STREAM SETTINGS", ORANGE, [
                ("fps", 5, 120, 60.0, "FPS / Update rate"),
            ]),
        ]

        for group_name, color, entries in SLIDER_GROUPS:
            grp = tk.LabelFrame(
                inner, text=f"  {group_name}  ",
                bg=BG2, fg=color, font=("Consolas", 9, "bold"),
                bd=1, relief="groove", padx=8, pady=6,
            )
            grp.pack(fill="x", padx=8, pady=6)

            for key, lo, hi, default, label in entries:
                row = tk.Frame(grp, bg=BG2)
                row.pack(fill="x", pady=2)

                tk.Label(row, text=f"{label:<18}", bg=BG2, fg=FG2,
                         font=("Consolas", 9), width=18, anchor="w").pack(side="left")

                sv = tk.StringVar(value=f"{default:8.2f}")
                self._sv[key] = sv
                tk.Label(row, textvariable=sv, bg=BG2, fg=color,
                         font=("Consolas", 9), width=9, anchor="e").pack(side="right", padx=4)

                s = tk.Scale(
                    row,
                    from_=lo, to=hi,
                    orient="horizontal",
                    resolution=(hi - lo) / 1000.0,
                    bg=BG2, fg=FG, troughcolor=SLIDER_TROUGH,
                    activebackground=SLIDER_ACTIVE,
                    highlightthickness=0, bd=0,
                    showvalue=False,
                    command=lambda val, k=key: self._on_slider(k, val),
                )
                s.set(default)
                s.pack(side="left", fill="x", expand=True)
                self._sliders[key] = s

    # ── Presets tab ───────────────────────────────────────────────────────────
    def _build_presets(self, parent):
        tk.Label(parent, text="Motion Presets", bg=BG,
                 fg=ACCENT, font=("Consolas", 12, "bold")).pack(pady=12)

        self._preset_var = tk.StringVar(value="None")

        frame = tk.Frame(parent, bg=BG)
        frame.pack()

        tk.Radiobutton(
            frame, text="None (use sliders)", variable=self._preset_var,
            value="None", bg=BG, fg=FG, selectcolor=BG3,
            activebackground=BG, font=("Consolas", 10),
            command=self._on_preset_change,
        ).pack(anchor="w", padx=20, pady=4)

        for name in PRESETS:
            tk.Radiobutton(
                frame, text=name, variable=self._preset_var,
                value=name, bg=BG, fg=FG, selectcolor=BG3,
                activebackground=BG, font=("Consolas", 10),
                command=self._on_preset_change,
            ).pack(anchor="w", padx=20, pady=4)

        desc = tk.Label(
            parent,
            text=(
                "NOTE: selecting a preset overrides sliders.\n"
                "Switch back to 'None' to resume manual slider control.\n\n"
                "To add your own preset: open simulator_gui.py and\n"
                "add a function + entry in the PRESETS dict at the top."
            ),
            bg=BG, fg=FG2, font=("Consolas", 9), justify="left",
        )
        desc.pack(pady=12, padx=20, anchor="w")

    # ── Preview tab ───────────────────────────────────────────────────────────
    def _build_preview(self, parent):
        left = tk.Frame(parent, bg=BG)
        left.pack(side="left", fill="both", expand=True, padx=8, pady=8)

        tk.Label(left, text="Live Pressure Map", bg=BG,
                 fg=ACCENT, font=("Consolas", 10, "bold")).pack()

        cw = FRAME_W * self.PREVIEW_SCALE
        ch = FRAME_H * self.PREVIEW_SCALE
        self._preview_canvas = tk.Canvas(
            left, width=cw, height=ch, bg="#000000",
            highlightthickness=1, highlightbackground=BG3,
        )
        self._preview_canvas.pack(pady=4)
        self._preview_img = None

        right = tk.Frame(parent, bg=BG)
        right.pack(side="left", fill="y", padx=8, pady=8)

        tk.Label(right, text="IMU Readings", bg=BG,
                 fg=ACCENT, font=("Consolas", 10, "bold")).pack()

        self._imu_lbl = tk.Label(
            right, text="", bg=BG2, fg=GREEN,
            font=("Consolas", 10), justify="left",
            padx=8, pady=8, relief="groove",
        )
        self._imu_lbl.pack(pady=4, fill="x")

        tk.Label(right, text="Stream Stats", bg=BG,
                 fg=ACCENT, font=("Consolas", 10, "bold")).pack(pady=(12, 0))

        self._stats_lbl = tk.Label(
            right, text="", bg=BG2, fg=YELLOW,
            font=("Consolas", 10), justify="left",
            padx=8, pady=8, relief="groove",
        )
        self._stats_lbl.pack(pady=4, fill="x")

    # ────────────────────────────────────────── CALLBACKS ────────────────────
    def _toggle_stream(self):
        if self.core.streaming:
            self.core.streaming = False
            self._btn_start.config(text="▶  START STREAM", bg=GREEN, fg="#11111b")
        else:
            self.core.t0 = time.time()
            self.core.frame_count = 0
            self.core.streaming = True
            self._btn_start.config(text="⏸  PAUSE STREAM", bg=ORANGE, fg="#11111b")

    def _reset_imu(self):
        self.core.reset_imu()
        for key, default in [
            ("gx", 0.0), ("gy", 0.0), ("gz", 0.0),
            ("ax", 0.0), ("ay", 0.0), ("az", 1.0),
            ("mx", 40.0), ("my", 0.0), ("mz", 30.0),
        ]:
            if key in self._sliders:
                self._sliders[key].set(default)

    def _toggle_record(self):
        if self.core.recording:
            self.core.stop_recording()
            self._btn_record.config(text="⏺  RECORD", bg=BG3, fg=FG)
            messagebox.showinfo("Recording", "Recording saved.")
        else:
            path = filedialog.asksaveasfilename(
                defaultextension=".csv",
                filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
                title="Save Recording As",
            )
            if path:
                self.core.start_recording(path)
                self._btn_record.config(text="⏹  STOP REC", bg=RED, fg="#11111b")

    def _start_replay(self):
        path = filedialog.askopenfilename(
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            title="Open Session Recording",
        )
        if not path:
            return
        n = self.core.load_replay(path)
        if n == 0:
            messagebox.showwarning("Replay", "No rows found in file.")
            return
        self._replay_idx = 0
        self._replay_active = True
        messagebox.showinfo("Replay", f"Loaded {n} rows. Replay active.")

    def _stop_replay(self):
        self._replay_active = False
        self.core.replay_rows = []

    def _on_slider(self, key: str, val: str):
        v = float(val)
        if key == "fps":
            self.core.fps = max(1.0, v)
        else:
            self.core.state[key] = v
        if key in self._sv:
            self._sv[key].set(f"{v:8.2f}")

    def _on_imu_mode_change(self):
        manual = (self._imu_mode_var.get() == "MANUAL")
        self.core.state["imu_manual"] = manual

    def _on_pressure_mode_change(self):
        manual = (self._press_mode_var.get() == "MANUAL")
        self.core.state["pressure_manual"] = manual

    def _on_preset_change(self):
        name = self._preset_var.get()
        self.core.preset_fn = PRESETS.get(name)

    # ────────────────────────────────────────── PERIODIC REFRESH ─────────────
    def _refresh(self):
        self._update_slider_labels()
        self._update_preview()
        self._update_stats()
        if self._replay_active:
            self._tick_replay()
        self.root.after(100, self._refresh)   # refresh GUI at 10 Hz to stay light

    def _update_slider_labels(self):
        """Sync numeric labels next to sliders with current core state."""
        state = self.core.state
        for key, sv in self._sv.items():
            if key == "fps":
                sv.set(f"{self.core.fps:8.2f}")
            elif key in state:
                sv.set(f"{float(state[key]):8.2f}")

    def _update_preview(self):
        """Render pressure matrix as coloured pixels on preview canvas."""
        mat = self.core.last_mat.astype(float)
        scale = self.PREVIEW_SCALE
        cw = FRAME_W * scale
        ch = FRAME_H * scale

        # Build RGBA image with rainbow colourmap approximation
        norm = mat / max(PRESSURE_MAX, 1)
        # Use a simple hue-based colour (red=high, blue=low)
        r = np.clip(norm * 2.0,        0, 1)
        g = np.clip(1 - abs(norm*2-1), 0, 1)
        b = np.clip(1 - norm * 2.0,    0, 1)

        # Upscale
        r_up = np.repeat(np.repeat(r, scale, axis=0), scale, axis=1)
        g_up = np.repeat(np.repeat(g, scale, axis=0), scale, axis=1)
        b_up = np.repeat(np.repeat(b, scale, axis=0), scale, axis=1)

        # Build PPM P6 bytes for tk.PhotoImage
        pixels = (np.stack([r_up, g_up, b_up], axis=-1) * 255).astype(np.uint8)
        ppm_header = f"P6\n{cw} {ch}\n255\n".encode()
        img_data = ppm_header + pixels.tobytes()

        img = tk.PhotoImage(width=cw, height=ch, data=img_data, format="PPM")
        self._preview_canvas.delete("all")
        self._preview_canvas.create_image(0, 0, anchor="nw", image=img)
        self._preview_img = img   # keep reference

    def _update_stats(self):
        """Update IMU labels and FPS/packet counter."""
        s = self.core.state
        now = time.time()
        dt = now - self._fps_last_time
        if dt >= 1.0:
            cnt = self.core.frame_count
            self._actual_fps = (cnt - self._fps_last_count) / dt
            self._fps_last_count = cnt
            self._fps_last_time = now

        imu_text = (
            f"gx = {s['gx']:8.2f} °/s\n"
            f"gy = {s['gy']:8.2f} °/s\n"
            f"gz = {s['gz']:8.2f} °/s\n\n"
            f"ax = {s['ax']:8.3f} g\n"
            f"ay = {s['ay']:8.3f} g\n"
            f"az = {s['az']:8.3f} g\n\n"
            f"mx = {s['mx']:8.2f} µT\n"
            f"my = {s['my']:8.2f} µT\n"
            f"mz = {s['mz']:8.2f} µT"
        )
        if hasattr(self, "_imu_lbl"):
            self._imu_lbl.config(text=imu_text)

        rec_badge = " ⏺REC" if self.core.recording else ""
        status = "STREAMING" if self.core.streaming else "PAUSED"
        stats_text = (
            f"Status  : {status}{rec_badge}\n"
            f"Packets : {self.core.frame_count}\n"
            f"Act. FPS: {self._actual_fps:.1f}\n"
            f"Tgt. FPS: {self.core.fps:.0f}\n"
            f"Dest    : {self.core.ip}:{self.core.port}"
        )
        if hasattr(self, "_stats_lbl"):
            self._stats_lbl.config(text=stats_text)

        bar_text = (
            f"  Frames: {self.core.frame_count}   "
            f"FPS: {self._actual_fps:.1f}/{self.core.fps:.0f}   "
            f"{'● REC' if self.core.recording else ''}   "
            f"{'▶ STREAM' if self.core.streaming else '⏸ PAUSED'}"
        )
        self._lbl_stats.config(text=bar_text)

    def _tick_replay(self):
        """Advance one row of replay data into core state."""
        rows = self.core.replay_rows
        if not rows or self._replay_idx >= len(rows):
            self._replay_active = False
            return
        row = rows[self._replay_idx]
        self._replay_idx += 1
        try:
            self.core.state.update({
                "gx": float(row["gx_dps"]), "gy": float(row["gy_dps"]), "gz": float(row["gz_dps"]),
                "ax": float(row["ax_g"]),   "ay": float(row["ay_g"]),   "az": float(row["az_g"]),
                "mx": float(row["mx_uT"]),  "my": float(row["my_uT"]),  "mz": float(row["mz_uT"]),
                "cx": float(row["cx"]),     "cy": float(row["cy"]),
                "radius":   float(row["radius"]),
                "strength": float(row["strength"]),
                "imu_manual":      True,
                "pressure_manual": True,
            })
        except (KeyError, ValueError):
            pass


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Grip-Pressure + IMU GUI Simulator"
    )
    parser.add_argument("--ip",   default="127.0.0.1", help="Destination IP for UDP packets")
    parser.add_argument("--port", type=int, default=12345, help="Destination UDP port")
    parser.add_argument("--fps",  type=float, default=60.0, help="Initial stream FPS")
    parser.add_argument("--auto-start", action="store_true", help="Start streaming immediately")
    args = parser.parse_args()

    core = SimulatorCore(args.ip, args.port, args.fps)

    streamer = UDPStreamer(core)
    streamer.start()

    root = tk.Tk()
    gui = GUIController(root, core)

    if args.auto_start:
        core.streaming = True
        gui._btn_start.config(text="⏸  PAUSE STREAM", bg=ORANGE, fg="#11111b")

    def on_close():
        core.quit_flag = True
        core.streaming = False
        core.stop_recording()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
