#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
viz_full_with_gyro_accel_mag.py — 32×64 grip-pressure visualizer + IMU (gyro/accel/mag)

Reads multiple packet types over the SAME UDP socket:
1) Pressure chunks (10 × 415B) -> assemble 32×64 frame (kept identical)
2) IMU packets:
   - b'\xAA\x55IM' + 9*int16 (BE) = gyroXYZ, accelXYZ, magXYZ
   - b'\xAA\x55GY' + 6*int16 (BE) = gyroXYZ, accelXYZ (fallback)

Run:
  python viz_full_with_gyro_accel_mag.py --ip 0.0.0.0 --port 12345 --buffer 800 --show-magnitude
"""

import argparse
import socket
import time
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from matplotlib.gridspec import GridSpec

# -----------------------------
# CLI
# -----------------------------
parser = argparse.ArgumentParser(description="32x64 UDP visualizer + IMU (gyro/accel/mag)")
parser.add_argument("--ip", default="0.0.0.0", help="IP to bind")
parser.add_argument("--port", type=int, default=12345, help="UDP port")
parser.add_argument("--buffer", type=int, default=800, help="IMU sliding window")
parser.add_argument("--show-magnitude", action="store_true", help="Plot magnitudes for gyro/accel/mag")
parser.add_argument("--save-csv", default="", help="Optional CSV log (IMU only)")
args, _ = parser.parse_known_args()

# -----------------------------
# Pressure config (unchanged core)
# -----------------------------
FRAME_W, FRAME_H = 64, 32
FRAME_SIZE = FRAME_W * FRAME_H

CHUNKS_PER_FRAME = 10
VALUES_PER_CHUNK = 205
BYTES_PER_PACKET = 415

UDP_DRAIN_PER_TICK = 140
ANIM_INTERVAL_MS = 10

# Timeouts
FRAME_TIMEOUT_S = 0.20
STREAM_SILENCE_RESET_S = 1.5

# Visualization scale
VMIN, VMAX = 0, 4095
USE_AUTO_CLIM = False

# Geometry transforms
APPLY_FLIP_LR = False
ROTATE_K = 2                      # 0,1,2,3
APPLY_HALF_MIRROR = True          # mirror left half only

# Endianness handling for pressure payload
ENDIAN_MODE = "AUTO"              # "AUTO" | "LE" | "BE"

# -----------------------------
# IMU packet formats
# -----------------------------
HDR_IM = b"\xAA\x55IM"            # gyro+accel+mag (preferred)
HDR_GY = b"\xAA\x55GY"            # gyro+accel (fallback)
LEN_IM = 4 + (9 * 2)              # 22 bytes
LEN_GY = 4 + (6 * 2)              # 16 bytes

GYRO_SF = 131.0                   # LSB/°/s (±250 dps)
ACC_SF  = 16384.0                 # LSB/g  (±2 g)
MAG_SF_uT = 0.15                  # µT/LSB (AK09916)

# -----------------------------
# UDP socket (non-blocking)
# -----------------------------
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((args.ip, args.port))
sock.setblocking(False)
print(f"[LISTEN] UDP {args.ip}:{args.port}")

# -----------------------------
# Pressure state
# -----------------------------
pending_chunks = [None] * CHUNKS_PER_FRAME  # each: np.ndarray('<u2', 205)
received_idxs = set()
frame_start_ts = None
last_packet_ts = time.time()

# stats
frames_ok = 0
frames_dropped = 0
smoothed_fps = 0.0
fps_window_start = time.time()
fps_frames = 0

chosen_endian = ENDIAN_MODE  # "AUTO" | "LE" | "BE"

# -----------------------------
# IMU state (deques)
# -----------------------------
N = args.buffer
start_time = time.time()

# Time buffers:
t_buf  = deque(maxlen=N)      # for gyro+accel (both IM/GY)
t_mag_buf = deque(maxlen=N)   # for magnetometer ONLY (IM packets)

# Data buffers
gx_buf = deque(maxlen=N); gy_buf = deque(maxlen=N); gz_buf = deque(maxlen=N)
ax_buf = deque(maxlen=N); ay_buf = deque(maxlen=N); az_buf = deque(maxlen=N)
mx_buf = deque(maxlen=N); my_buf = deque(maxlen=N); mz_buf = deque(maxlen=N)

gmag_buf = deque(maxlen=N); amag_buf = deque(maxlen=N); mmag_buf = deque(maxlen=N)

# Optional CSV logging
csv_fp = None
if args.save_csv:
    csv_fp = open(args.save_csv, "w", buffering=1)
    csv_fp.write("time,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,mx_uT,my_uT,mz_uT\n")

# -----------------------------
# Matplotlib layout
# -----------------------------
plt.rcParams["toolbar"] = "toolbar2"

# Use constrained_layout to avoid tight_layout warning
fig = plt.figure(figsize=(11.0, 9.2), constrained_layout=True)
gs = GridSpec(nrows=4, ncols=1, height_ratios=[4.3, 1.1, 1.1, 1.1], figure=fig)

# (1) Pressure heatmap
axP = fig.add_subplot(gs[0, 0])
mat0 = np.zeros((FRAME_H, FRAME_W), dtype=np.uint16)
im = axP.imshow(mat0, cmap="rainbow", vmin=VMIN, vmax=VMAX, aspect="auto")
cbar = fig.colorbar(im, ax=axP)
cbar.set_label("ADC / Pressure")

title = axP.set_title("32×64 Grip Pressure Visualization - 0.0 FPS")
axP.set_xlabel("X (cols)")
axP.set_ylabel("Y (rows)")
axP.set_xlim(-0.5, FRAME_W - 0.5)
axP.set_ylim(FRAME_H - 0.5, -0.5)  # put (0,0) at top-left

last_hash = None
unchanged_count = 0
STUCK_THRESH = 30
status_txt = axP.text(0.01, 0.02, "", color="w", transform=axP.transAxes,
                      fontsize=10, bbox=dict(facecolor="black", alpha=0.4, pad=3))

# (2) Gyro plot
axG = fig.add_subplot(gs[1, 0])
l_gx, = axG.plot([], [], label="gx (°/s)")
l_gy, = axG.plot([], [], label="gy (°/s)")
l_gz, = axG.plot([], [], label="gz (°/s)")
if args.show_magnitude:
    l_gmag, = axG.plot([], [], label="|g|")
axG.set_xlabel("Time (s)")
axG.set_ylabel("Angular rate (°/s)")
axG.grid(True)
axG.legend(loc="upper right")

# (3) Accel plot
axA = fig.add_subplot(gs[2, 0])
l_ax, = axA.plot([], [], label="ax (g)")
l_ay, = axA.plot([], [], label="ay (g)")
l_az, = axA.plot([], [], label="az (g)")
if args.show_magnitude:
    l_amag, = axA.plot([], [], label="|a|")
axA.set_xlabel("Time (s)")
axA.set_ylabel("Acceleration (g)")
axA.grid(True)
axA.legend(loc="upper right")

# (4) Magnetometer plot
axM = fig.add_subplot(gs[3, 0])
l_mx, = axM.plot([], [], label="mx (µT)")
l_my, = axM.plot([], [], label="my (µT)")
l_mz, = axM.plot([], [], label="mz (µT)")
if args.show_magnitude:
    l_mmag, = axM.plot([], [], label="|m|")
axM.set_xlabel("Time (s)")
axM.set_ylabel("Magnetic field (µT)")
axM.grid(True)
axM.legend(loc="upper right")

# -----------------------------
# Helpers (pressure)
# -----------------------------
def reset_frame_buffers():
    global pending_chunks, received_idxs, frame_start_ts
    pending_chunks = [None] * CHUNKS_PER_FRAME
    received_idxs.clear()
    frame_start_ts = None

def choose_endianness(all_le: np.ndarray) -> str:
    le_max = int(all_le.max())
    if le_max <= 4095:
        return "LE"
    be_max = int(all_le.byteswap().max())
    if be_max <= 4095:
        return "BE"
    return "LE"

def build_and_show_frame():
    """Assemble the pressure frame, apply transforms, update heatmap & FPS."""
    global frames_ok, smoothed_fps, fps_frames, fps_window_start, chosen_endian

    all_le = np.concatenate(pending_chunks)[:FRAME_SIZE]

    if chosen_endian == "AUTO":
        chosen_endian = choose_endianness(all_le)

    all_vals = all_le.byteswap() if chosen_endian == "BE" else all_le
    mat = all_vals.reshape(FRAME_H, FRAME_W)

    if APPLY_HALF_MIRROR:
        mid = FRAME_W // 2
        left  = mat[:, :mid][:, ::-1]   # flip columns 0..31
        right = mat[:, mid:]            # keep 32..63 as-is
        mat = np.concatenate((left, right), axis=1)

    if APPLY_FLIP_LR:
        mat = np.fliplr(mat)
    if ROTATE_K:
        mat = np.rot90(mat, k=ROTATE_K)

    if USE_AUTO_CLIM:
        lo, hi = np.percentile(mat, [5, 95])
        if hi <= lo: hi = lo + 1
        im.set_clim(lo, hi)
    else:
        im.set_clim(VMIN, VMAX)

    im.set_data(mat)

    # Stuck detector
    h = int(np.uint32(mat.sum() * 2654435761 & 0xFFFFFFFF))
    global last_hash, unchanged_count
    if last_hash is None or h != last_hash:
        unchanged_count = 0
        last_hash = h
    else:
        unchanged_count += 1

    status_txt.set_text(f"⚠ STUCK {unchanged_count} frames" if unchanged_count >= STUCK_THRESH else "")

    # FPS (smoothed)
    now = time.time()
    global fps_frames, fps_window_start, smoothed_fps
    fps_frames += 1
    dt = now - fps_window_start
    if dt >= 0.5:
        inst = fps_frames / dt
        smoothed_fps = inst if smoothed_fps == 0.0 else (0.7 * smoothed_fps + 0.3 * inst)
        fps_frames = 0
        fps_window_start = now

    global frames_ok
    frames_ok += 1
    title.set_text(f"32×64 Grip Pressure Visualization - {smoothed_fps:.1f} FPS")

# -----------------------------
# Packet routing
# -----------------------------
def route_packet(pkt: bytes):
    """Route incoming packet to either pressure or IMU path."""
    # 1) IMU with mag?
    if len(pkt) >= LEN_IM and pkt[:4] == HDR_IM:
        try:
            gx_i, gy_i, gz_i, ax_i, ay_i, az_i, mx_i, my_i, mz_i = struct.unpack(">9h", pkt[4:22])
        except struct.error:
            return
        t = time.time() - start_time
        gx, gy, gz = gx_i / GYRO_SF, gy_i / GYRO_SF, gz_i / GYRO_SF
        ax, ay, az = ax_i / ACC_SF , ay_i / ACC_SF , az_i / ACC_SF
        mx, my, mz = mx_i * MAG_SF_uT, my_i * MAG_SF_uT, mz_i * MAG_SF_uT

        # time stamps
        t_buf.append(t)          # gyro/accel timeline
        t_mag_buf.append(t)      # mag timeline

        # data
        gx_buf.append(gx); gy_buf.append(gy); gz_buf.append(gz)
        ax_buf.append(ax); ay_buf.append(ay); az_buf.append(az)
        mx_buf.append(mx); my_buf.append(my); mz_buf.append(mz)

        if args.show_magnitude:
            gmag_buf.append((gx*gx + gy*gy + gz*gz) ** 0.5)
            amag_buf.append((ax*ax + ay*ay + az*az) ** 0.5)
            mmag_buf.append((mx*mx + my*my + mz*mz) ** 0.5)
        if csv_fp:
            csv_fp.write(f"{t:.6f},{gx:.6f},{gy:.6f},{gz:.6f},{ax:.6f},{ay:.6f},{az:.6f},{mx:.6f},{my:.6f},{mz:.6f}\n")
        return

    # 2) IMU without mag (old GY packet)
    if len(pkt) >= LEN_GY and pkt[:4] == HDR_GY:
        try:
            gx_i, gy_i, gz_i, ax_i, ay_i, az_i = struct.unpack(">6h", pkt[4:16])
        except struct.error:
            return
        t = time.time() - start_time
        gx, gy, gz = gx_i / GYRO_SF, gy_i / GYRO_SF, gz_i / GYRO_SF
        ax, ay, az = ax_i / ACC_SF , ay_i / ACC_SF , az_i / ACC_SF

        t_buf.append(t)  # only gyro/accel timeline grows here

        gx_buf.append(gx); gy_buf.append(gy); gz_buf.append(gz)
        ax_buf.append(ax); ay_buf.append(ay); az_buf.append(az)

        if args.show_magnitude:
            gmag_buf.append((gx*gx + gy*gy + gz*gz) ** 0.5)
            amag_buf.append((ax*ax + ay*ay + az*az) ** 0.5)
        if csv_fp:
            csv_fp.write(f"{t:.6f},{gx:.6f},{gy:.6f},{gz:.6f},{ax:.6f},{ay:.6f},{az:.6f},,,\n")
        return

    # 3) Pressure chunk?
    if len(pkt) == BYTES_PER_PACKET and pkt[0] == 0xAA and pkt[1] == 0x55 and pkt[-2] == 0x55 and pkt[-1] == 0xAA:
        idx = pkt[2]
        if idx >= CHUNKS_PER_FRAME:
            return
        payload = pkt[3:-2]  # 410 bytes
        if len(payload) != 410:
            return
        vals_le = np.frombuffer(payload, dtype="<u2", count=VALUES_PER_CHUNK)
        global frame_start_ts
        if not received_idxs:
            frame_start_ts = time.time()
        if idx not in received_idxs:
            pending_chunks[idx] = vals_le
            received_idxs.add(idx)
        return

    # else: unknown packet type -> ignore

# -----------------------------
# Animation tick
# -----------------------------
def on_timer(_):
    """Drain UDP, assemble frames with timeouts, update all plots."""
    global frames_dropped, last_packet_ts

    # Drain multiple packets per tick
    for _ in range(UDP_DRAIN_PER_TICK):
        try:
            pkt, _ = sock.recvfrom(4096)
            last_packet_ts = time.time()
        except BlockingIOError:
            break
        route_packet(pkt)

        # If a full pressure frame is ready, draw immediately
        if len(received_idxs) == CHUNKS_PER_FRAME and all(ch is not None for ch in pending_chunks):
            build_and_show_frame()
            reset_frame_buffers()

    # Handle pressure partial-frame timeout
    if received_idxs and frame_start_ts is not None:
        if (time.time() - frame_start_ts) > FRAME_TIMEOUT_S:
            frames_dropped += 1
            reset_frame_buffers()

    # Reset on long silence
    if (time.time() - last_packet_ts) > STREAM_SILENCE_RESET_S:
        reset_frame_buffers()

    # Update IMU lines
    if len(t_buf) > 1:
        # Gyro & Accel use t_buf
        l_gx.set_data(t_buf, gx_buf); l_gy.set_data(t_buf, gy_buf); l_gz.set_data(t_buf, gz_buf)
        axG.relim(); axG.autoscale_view()
        l_ax.set_data(t_buf, ax_buf); l_ay.set_data(t_buf, ay_buf); l_az.set_data(t_buf, az_buf)
        axA.relim(); axA.autoscale_view()
        if args.show_magnitude:
            l_gmag.set_data(t_buf, gmag_buf); l_amag.set_data(t_buf, amag_buf)

    # Magnetometer uses its own time buffer (avoid length mismatch)
    if len(t_mag_buf) > 1:
        l_mx.set_data(t_mag_buf, mx_buf); l_my.set_data(t_mag_buf, my_buf); l_mz.set_data(t_mag_buf, mz_buf)
        axM.relim(); axM.autoscale_view()
        if args.show_magnitude and len(mmag_buf) > 1:
            l_mmag.set_data(t_mag_buf, mmag_buf)

    return [im, l_gx, l_gy, l_gz, l_ax, l_ay, l_az, l_mx, l_my, l_mz]

# -----------------------------
# Run
# -----------------------------
ani = FuncAnimation(fig, on_timer, interval=ANIM_INTERVAL_MS, blit=False)
print("[RUN] Close the window to exit.")
try:
    plt.show()
finally:
    try:
        sock.close()
    except Exception:
        pass
    if csv_fp:
        csv_fp.close()
