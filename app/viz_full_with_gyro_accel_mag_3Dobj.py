#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
viz_full_with_gyro_accel_mag_3Dobj.py — 32×64 grip-pressure visualizer + IMU (gyro/accel/mag) + optional 3D shape (cylinder)

Reads multiple packet types over the SAME UDP socket:
1) Pressure chunks (10 × 415B) -> assemble 32×64 frame (kept identical)
2) IMU packets:
   - b'\xAA\x55IM' + 9*int16 (BE) = gyroXYZ, accelXYZ, magXYZ
   - b'\xAA\x55GY' + 6*int16 (BE) = gyroXYZ, accelXYZ (fallback)

Run:
    python viz_full_with_gyro_accel_mag_3Dobj.py --ip 0.0.0.0 --port 12345 --buffer 800 --show-magnitude --show-3d
"""

import argparse
import socket
import time
import struct
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from collections import deque
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (imported for 3D projection)
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# -----------------------------
# CLI
# -----------------------------
parser = argparse.ArgumentParser(description="32x64 UDP visualizer + IMU (gyro/accel/mag)")
parser.add_argument("--ip", default="0.0.0.0", help="IP to bind")
parser.add_argument("--port", type=int, default=12345, help="UDP port")
parser.add_argument("--buffer", type=int, default=800, help="IMU sliding window")
parser.add_argument("--show-magnitude", action="store_true", help="Plot magnitudes for gyro/accel/mag")
parser.add_argument("--save-csv", default="", help="Optional CSV log (IMU only)")
parser.add_argument("--show-3d", action="store_true", help="Show 3D orientation cube (computed from gyro/accel/mag)")
parser.add_argument("--save-session-json", default="", help="Optional NDJSON log for pressure+IMU replay in web viewer")
args, _ = parser.parse_known_args()

# -----------------------------
# Pressure config (unchanged core)
# -----------------------------
FRAME_W, FRAME_H = 64, 32
FRAME_SIZE = FRAME_W * FRAME_H

CHUNKS_PER_FRAME = 10
VALUES_PER_CHUNK = 205
BYTES_PER_PACKET = 415

UDP_DRAIN_PER_TICK = 64
ANIM_INTERVAL_MS = 33
IMU_AUTOSCALE_EVERY_N = 3

# Timeouts
FRAME_TIMEOUT_S = 0.20
STREAM_SILENCE_RESET_S = 1.5

# Visualization scale
VMIN, VMAX = 0, 4095
USE_AUTO_CLIM = False


# -----------------------------
# Geometry transforms
# -----------------------------
APPLY_FLIP_LR = False
ROTATE_K = 2                      # 0,1,2,3
APPLY_HALF_MIRROR = False          # mirror left half only

# -----------------------------
# Sensor/cylinder geometry config (edit here)
# -----------------------------

# ====== SENSOR/CYLINDER CONFIGURATION ======
SENSOR_ROWS = 32                  # Number of sensor rows (height direction)
SENSOR_COLS = 64                  # Number of sensor columns (circumference direction)
SENSOR_COVERAGE_DEG = 300         # Typical: 300 (sensor wrap angle in degrees)
CABLE_GAP_DEG = 60                # Typical: 60 (gap angle in degrees)
CABLE_GAP_CENTER_DEG = 180        # Typical: 180 (gap centered at back, Y negative)
GAP_COLOR = "black"               # Color for cable gap (edit for appearance)
CYL_RADIUS = 0.5                  # Cylinder radius (edit for size)
CYL_HEIGHT = 2.0                  # Cylinder height (edit for size)
CYL_SIDES = 72                    # Number of mesh sides (higher = smoother, but slower)
PRESSURE_CMAP = "rainbow"         # Colormap for pressure overlay (edit for style)
SHOW_3D_MESH_EDGES = False        # Set True for visible mesh lines (costs more GPU/CPU)
# VMIN, VMAX already defined above for colormap scale
#
# To make the sensor overlay always visible from the default view:
# - Keep CABLE_GAP_CENTER_DEG = 180 (gap at back)
# - SENSOR_COVERAGE_DEG + CABLE_GAP_DEG should be 360
#
# If you want to test full coverage, set SENSOR_COVERAGE_DEG = 360, CABLE_GAP_DEG = 0

# Comments:
# - Adjust SENSOR_COVERAGE_DEG to match sensor wrap angle
# - Adjust CABLE_GAP_DEG and CABLE_GAP_CENTER_DEG to move/resize the cable gap
# - Adjust CYL_RADIUS and CYL_HEIGHT for cylinder size
# - Adjust PRESSURE_CMAP, VMIN, VMAX for color mapping

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
imu_plot_tick = 0

chosen_endian = ENDIAN_MODE  # "AUTO" | "LE" | "BE"

###############################
# (Former debug counters removed)
###############################

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

raw_dump_fp = None  # (placeholder, debug dump removed)

# Optional NDJSON recording (one JSON object per line for streaming-safe writes).
session_fp = None
session_frame_idx = 0
recording_enabled = False
if args.save_session_json:
    session_fp = open(args.save_session_json, "w", buffering=1)
    session_fp.write(json.dumps({
        "type": "meta",
        "schema": "kinesiology.pressure_imu.ndjson.v1",
        "created_unix_s": time.time(),
        "frame_shape": [FRAME_H, FRAME_W],
        "pressure_dtype": "uint16",
        "pressure_layout": "row-major-flat",
        "imu_units": {"gyro": "deg_s", "accel": "g", "mag": "uT"},
    }) + "\n")
    recording_enabled = True

latest_imu = {
    "t_rel_s": None,
    "gx": None,
    "gy": None,
    "gz": None,
    "ax": None,
    "ay": None,
    "az": None,
    "mx": None,
    "my": None,
    "mz": None,
    "has_mag": False,
}

latest_orientation = {
    "roll_deg": 0.0,
    "pitch_deg": 0.0,
    "yaw_deg": 0.0,
}


def record_display_frame(mat: np.ndarray):
    """Write one display frame plus latest IMU/orientation into NDJSON log."""
    global session_frame_idx
    if session_fp is None or not recording_enabled:
        return

    entry = {
        "type": "frame",
        "frame_idx": session_frame_idx,
        "t_rel_s": time.time() - start_time,
        "t_unix_s": time.time(),
        "pressure_u16_flat": mat.astype(np.uint16, copy=False).reshape(-1).tolist(),
        "imu": dict(latest_imu),
        "orientation_deg": dict(latest_orientation),
    }
    session_fp.write(json.dumps(entry, separators=(",", ":")) + "\n")
    session_frame_idx += 1

    # Keep data durable on disk during longer runs without flushing every frame.
    if (session_frame_idx % 30) == 0:
        session_fp.flush()

# -----------------------------
# Matplotlib layout
# -----------------------------
plt.rcParams["toolbar"] = "toolbar2"

# Use constrained_layout to avoid tight_layout warning
if args.show_3d:
    # Wider figure; left column identical sizing for pressure + time-series, right column reserved for 3D object.
    fig = plt.figure(figsize=(15.5, 9.2), constrained_layout=True)
    gs = GridSpec(nrows=4, ncols=2, height_ratios=[4.3, 1.1, 1.1, 1.1], width_ratios=[5.0, 2.2], figure=fig)
else:
    fig = plt.figure(figsize=(11.0, 9.2), constrained_layout=True)
    gs = GridSpec(nrows=4, ncols=1, height_ratios=[4.3, 1.1, 1.1, 1.1], figure=fig)


def _safe_disable_layout_engine(*_args):
    # Prevent constrained-layout callbacks from firing during teardown.
    try:
        fig.set_layout_engine("none")
    except Exception:
        pass


fig.canvas.mpl_connect("close_event", _safe_disable_layout_engine)

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

# Optional recording toggle button (works when --save-session-json is provided).
rec_btn = None
if session_fp is not None:
    ax_rec_btn = fig.add_axes([0.86, 0.94, 0.12, 0.045])
    rec_btn = Button(ax_rec_btn, "REC: ON", color="#7f1111", hovercolor="#a01818")

    def _toggle_recording(_event):
        global recording_enabled
        recording_enabled = not recording_enabled
        rec_btn.label.set_text("REC: ON" if recording_enabled else "REC: OFF")

    rec_btn.on_clicked(_toggle_recording)

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

# (5) Optional 3D orientation
if args.show_3d:
    # 3D axis spans all rows in right column without affecting pressure size.
    ax3d = fig.add_subplot(gs[:, 1], projection='3d')
    ax3d.set_title("3D Orientation (Cylinder)")
    lim_xy = CYL_RADIUS * 2.0
    lim_z = CYL_HEIGHT * 0.7
    ax3d.set_xlim([-lim_xy, lim_xy]); ax3d.set_ylim([-lim_xy, lim_xy]); ax3d.set_zlim([-lim_z, lim_z])
    ax3d.set_xlabel("X"); ax3d.set_ylabel("Y"); ax3d.set_zlabel("Z")
    ax3d.set_box_aspect((1.0, 1.0, CYL_HEIGHT / max(CYL_RADIUS * 2.0, 1e-6)))

    # --- Full cylinder mesh generation with explicit sensor/gap split ---
    def _normalize_deg(a):
        return a % 360.0

    def _angular_distance_deg(a, b):
        d = (a - b + 180.0) % 360.0 - 180.0
        return abs(d)

    def _is_gap_angle(theta_deg, gap_center_deg, gap_deg):
        return _angular_distance_deg(theta_deg, gap_center_deg) < (gap_deg * 0.5)

    def build_cylinder_surfaces(radius, height, n_rows, n_sides, gap_center_deg, gap_deg):
        """
        Build one full cylinder side mesh, then split faces into:
        - sensor-covered faces
        - cable-gap faces
        """
        # Use n_rows+1 vertices so we get exactly n_rows face bands (one per sensor row).
        z_vals = np.linspace(-height * 0.5, height * 0.5, n_rows + 1)
        theta_edges = np.linspace(0.0, 2.0 * np.pi, n_sides + 1)

        verts = []
        for z in z_vals:
            for th in theta_edges:
                verts.append([radius * np.cos(th), radius * np.sin(th), z])
        verts = np.asarray(verts, dtype=float)

        def vidx(i_row, j_col):
            return i_row * (n_sides + 1) + j_col

        all_faces = []
        face_row_idx = []
        face_theta_center = []
        sensor_face_idx = []
        gap_face_idx = []

        for i in range(n_rows):
            for j in range(n_sides):
                f = [
                    vidx(i, j),
                    vidx(i, j + 1),
                    vidx(i + 1, j + 1),
                    vidx(i + 1, j),
                ]
                all_faces.append(f)
                all_idx = len(all_faces) - 1
                theta_c_deg = _normalize_deg((j + 0.5) * (360.0 / n_sides))
                face_theta_center.append(theta_c_deg)
                face_row_idx.append(i)
                if _is_gap_angle(theta_c_deg, gap_center_deg, gap_deg):
                    gap_face_idx.append(all_idx)
                else:
                    sensor_face_idx.append(all_idx)

        return (
            verts,
            all_faces,
            np.asarray(face_row_idx, dtype=int),
            np.asarray(face_theta_center, dtype=float),
            np.asarray(sensor_face_idx, dtype=int),
            np.asarray(gap_face_idx, dtype=int),
        )

    def pressure_to_sensor_facecolors(mat, cmap_name, vmin, vmax, face_rows, face_theta_deg):
        """
        Convert 32x64 pressure matrix to RGBA colors for sensor faces only.
        Mapping:
        - matrix rows -> cylinder height direction
        - matrix cols -> sensor angular coverage only (gap excluded)
        """
        arr = np.asarray(mat, dtype=float)
        arr = np.clip(arr, vmin, vmax)
        denom = max(vmax - vmin, 1e-9)
        norm = (arr - vmin) / denom
        cmap_obj = plt.colormaps[cmap_name]

        # Sensor angular start/end where columns 0..63 are mapped.
        sensor_start_deg = _normalize_deg(CABLE_GAP_CENTER_DEG + (CABLE_GAP_DEG * 0.5))
        sensor_span_deg = SENSOR_COVERAGE_DEG

        rows_idx = np.clip(face_rows, 0, SENSOR_ROWS - 1).astype(int)
        rel = np.mod(face_theta_deg - sensor_start_deg, 360.0)
        rel = np.minimum(rel, sensor_span_deg - 1e-6)
        cols_idx = np.clip(((rel / sensor_span_deg) * SENSOR_COLS).astype(int), 0, SENSOR_COLS - 1)
        return cmap_obj(norm[rows_idx, cols_idx])

    (
        cyl_verts,
        cyl_faces_all,
        face_row_all,
        face_theta_all,
        sensor_face_idx,
        gap_face_idx,
    ) = build_cylinder_surfaces(
        CYL_RADIUS,
        CYL_HEIGHT,
        SENSOR_ROWS,
        CYL_SIDES,
        CABLE_GAP_CENTER_DEG,
        CABLE_GAP_DEG,
    )

    sensor_faces = [cyl_faces_all[i] for i in sensor_face_idx]
    gap_faces = [cyl_faces_all[i] for i in gap_face_idx]
    sensor_rows_for_faces = face_row_all[sensor_face_idx]
    sensor_thetas_for_faces = face_theta_all[sensor_face_idx]

    edge_color = 'k' if SHOW_3D_MESH_EDGES else 'none'
    edge_width = 0.2 if SHOW_3D_MESH_EDGES else 0.0

    # Sensor surface (dynamic facecolors from pressure map).
    sensor_poly = Poly3DCollection(
        [cyl_verts[f] for f in sensor_faces],
        facecolors=np.tile((0.1, 0.1, 0.1, 1.0), (len(sensor_faces), 1)),
        edgecolors=edge_color,
        linewidths=edge_width,
    )
    ax3d.add_collection3d(sensor_poly)

    # Gap surface (static black vertical strip).
    gap_poly = Poly3DCollection(
        [cyl_verts[f] for f in gap_faces],
        facecolors=GAP_COLOR,
        edgecolors=edge_color,
        linewidths=edge_width,
    )
    ax3d.add_collection3d(gap_poly)

    # Keep backward-compatible name used elsewhere.
    shape_poly = sensor_poly

    # Orientation state
    roll_deg = 0.0
    pitch_deg = 0.0
    yaw_deg = 0.0
    _last_imu_ts = None
    # Smoothing / filter parameters
    ALPHA_RP = 0.98  # complementary filter weight for roll/pitch
    ALPHA_YAW = 0.95

    def _normalize_angle(a):
        if a >= 180.0 or a < -180.0:
            a = ((a + 180.0) % 360.0) - 180.0
        return a

    def _update_orientation_state(gx, gy, gz, ax_v, ay_v, az_v, mx_v=None, my_v=None, mz_v=None):
        """Update global roll/pitch/yaw using complementary filter.
        gx,gy,gz: deg/s; ax_v..az_v: g; m*: µT
        """
        global roll_deg, pitch_deg, yaw_deg, _last_imu_ts
        now = time.time()
        if _last_imu_ts is None:
            _last_imu_ts = now
            return
        dt = now - _last_imu_ts
        _last_imu_ts = now
        if dt <= 0.0:
            return
        dt = min(dt, 0.2)  # clamp to avoid huge jumps

        # Accelerometer-only angles (deg)
        # protect against division issues
        ax_clip, ay_clip, az_clip = ax_v, ay_v, az_v if az_v != 0 else 1e-6
        roll_acc = np.degrees(np.arctan2(ay_clip, az_clip))
        pitch_acc = np.degrees(np.arctan2(-ax_clip, np.sqrt(ay_clip*ay_clip + az_clip*az_clip)))

        # Gyro integration
        roll_gyro = roll_deg + gx * dt
        pitch_gyro = pitch_deg + gy * dt
        yaw_gyro = yaw_deg + gz * dt

        # Complementary for roll/pitch
        roll_deg = ALPHA_RP * roll_gyro + (1.0 - ALPHA_RP) * roll_acc
        pitch_deg = ALPHA_RP * pitch_gyro + (1.0 - ALPHA_RP) * pitch_acc

        # Yaw using magnetometer if present, else pure gyro integration
        if mx_v is not None and my_v is not None and mz_v is not None:
            # Compute tilt-compensated mag
            pr = np.radians(pitch_deg)
            rr = np.radians(roll_deg)
            hx = mx_v * np.cos(pr) - mz_v * np.sin(pr)
            hy = my_v * np.cos(rr) + mx_v * np.sin(rr) * np.sin(pr) - mz_v * np.sin(rr) * np.cos(pr)
            yaw_mag = np.degrees(np.arctan2(hy, hx))
            yaw_deg = ALPHA_YAW * yaw_gyro + (1.0 - ALPHA_YAW) * yaw_mag
        else:
            yaw_deg = yaw_gyro

        roll_deg = _normalize_angle(roll_deg)
        pitch_deg = _normalize_angle(pitch_deg)
        yaw_deg = _normalize_angle(yaw_deg)
        latest_orientation["roll_deg"] = roll_deg
        latest_orientation["pitch_deg"] = pitch_deg
        latest_orientation["yaw_deg"] = yaw_deg
        # No return; modifies closure vars

    def _update_cube_artist():
        # Rotation matrices in Z-Y-X order
        cr, sr = np.cos(np.radians(roll_deg)), np.sin(np.radians(roll_deg))
        cp, sp = np.cos(np.radians(pitch_deg)), np.sin(np.radians(pitch_deg))
        cy, sy = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
        R_x = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
        R_y = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
        R_z = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
        R = R_z @ R_y @ R_x
        verts_rot = (R @ cyl_verts.T).T
        sensor_poly.set_verts([verts_rot[f] for f in sensor_faces])
        gap_poly.set_verts([verts_rot[f] for f in gap_faces])
        ax3d.set_xlim([-lim_xy, lim_xy]); ax3d.set_ylim([-lim_xy, lim_xy]); ax3d.set_zlim([-lim_z, lim_z])
        ax3d.view_init(elev=20., azim=45.)
        return sensor_poly
else:
    # Placeholders so references won't break if flag off
    shape_poly = None
    sensor_poly = None
    gap_poly = None
    def _update_orientation_state(*_args, **_kwargs):
        return
    def _update_cube_artist():
        return None

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
    """Assemble the pressure frame, apply transforms, update heatmap, FPS, and 3D cylinder overlay."""
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

    # Persist exactly what is shown on screen (after transforms).
    record_display_frame(mat)

    # --- Update 3D cylinder overlay ---
    if args.show_3d and sensor_poly is not None:
        sensor_colors = pressure_to_sensor_facecolors(
            mat,
            PRESSURE_CMAP,
            VMIN,
            VMAX,
            sensor_rows_for_faces,
            sensor_thetas_for_faces,
        )
        sensor_poly.set_facecolor(sensor_colors)

    # Stuck detector
    h = int(np.uint32(mat.sum() * 2654435761 & 0xFFFFFFFF))
    global last_hash, unchanged_count
    if last_hash is None or h != last_hash:
        unchanged_count = 0
        last_hash = h
    else:
        unchanged_count += 1

    # Simple status overlay (stuck only)
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
    # 1) IMU with mag
    if len(pkt) >= LEN_IM and pkt[:4] == HDR_IM:
        try:
            gx_i, gy_i, gz_i, ax_i, ay_i, az_i, mx_i, my_i, mz_i = struct.unpack(">9h", pkt[4:22])
        except struct.error:
            return
        t = time.time() - start_time
        gx, gy, gz = gx_i / GYRO_SF, gy_i / GYRO_SF, gz_i / GYRO_SF
        ax, ay, az = ax_i / ACC_SF, ay_i / ACC_SF, az_i / ACC_SF
        mx, my, mz = mx_i * MAG_SF_uT, my_i * MAG_SF_uT, mz_i * MAG_SF_uT
        latest_imu["t_rel_s"] = t
        latest_imu["gx"] = gx; latest_imu["gy"] = gy; latest_imu["gz"] = gz
        latest_imu["ax"] = ax; latest_imu["ay"] = ay; latest_imu["az"] = az
        latest_imu["mx"] = mx; latest_imu["my"] = my; latest_imu["mz"] = mz
        latest_imu["has_mag"] = True
        t_buf.append(t); t_mag_buf.append(t)
        gx_buf.append(gx); gy_buf.append(gy); gz_buf.append(gz)
        ax_buf.append(ax); ay_buf.append(ay); az_buf.append(az)
        mx_buf.append(mx); my_buf.append(my); mz_buf.append(mz)
        if args.show_magnitude:
            gmag_buf.append((gx*gx + gy*gy + gz*gz) ** 0.5)
            amag_buf.append((ax*ax + ay*ay + az*az) ** 0.5)
            mmag_buf.append((mx*mx + my*my + mz*mz) ** 0.5)
        if csv_fp:
            csv_fp.write(f"{t:.6f},{gx:.6f},{gy:.6f},{gz:.6f},{ax:.6f},{ay:.6f},{az:.6f},{mx:.6f},{my:.6f},{mz:.6f}\n")
        _update_orientation_state(gx, gy, gz, ax, ay, az, mx, my, mz)
        return
    # 2) IMU without mag
    if len(pkt) >= LEN_GY and pkt[:4] == HDR_GY:
        try:
            gx_i, gy_i, gz_i, ax_i, ay_i, az_i = struct.unpack(">6h", pkt[4:16])
        except struct.error:
            return
        t = time.time() - start_time
        gx, gy, gz = gx_i / GYRO_SF, gy_i / GYRO_SF, gz_i / GYRO_SF
        ax, ay, az = ax_i / ACC_SF, ay_i / ACC_SF, az_i / ACC_SF
        latest_imu["t_rel_s"] = t
        latest_imu["gx"] = gx; latest_imu["gy"] = gy; latest_imu["gz"] = gz
        latest_imu["ax"] = ax; latest_imu["ay"] = ay; latest_imu["az"] = az
        latest_imu["mx"] = None; latest_imu["my"] = None; latest_imu["mz"] = None
        latest_imu["has_mag"] = False
        t_buf.append(t)
        gx_buf.append(gx); gy_buf.append(gy); gz_buf.append(gz)
        ax_buf.append(ax); ay_buf.append(ay); az_buf.append(az)
        if args.show_magnitude:
            gmag_buf.append((gx*gx + gy*gy + gz*gz) ** 0.5)
            amag_buf.append((ax*ax + ay*ay + az*az) ** 0.5)
        if csv_fp:
            csv_fp.write(f"{t:.6f},{gx:.6f},{gy:.6f},{gz:.6f},{ax:.6f},{ay:.6f},{az:.6f},,,\n")
        _update_orientation_state(gx, gy, gz, ax, ay, az)
        return
    # 3) Pressure chunk
    if len(pkt) == BYTES_PER_PACKET and pkt[0] == 0xAA and pkt[1] == 0x55 and pkt[-2] == 0x55 and pkt[-1] == 0xAA:
        idx = pkt[2]
        if idx >= CHUNKS_PER_FRAME:
            return
        payload = pkt[3:-2]
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
    # else ignore unknown


# -----------------------------
# Animation tick
# -----------------------------
def on_timer(_):
    """Drain UDP, assemble frames with timeouts, update all plots."""
    global frames_dropped, last_packet_ts, imu_plot_tick

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
    imu_plot_tick += 1
    if len(t_buf) > 1 and (imu_plot_tick % IMU_AUTOSCALE_EVERY_N == 0):
        # Gyro & Accel use t_buf
        l_gx.set_data(t_buf, gx_buf); l_gy.set_data(t_buf, gy_buf); l_gz.set_data(t_buf, gz_buf)
        axG.relim(); axG.autoscale_view()
        l_ax.set_data(t_buf, ax_buf); l_ay.set_data(t_buf, ay_buf); l_az.set_data(t_buf, az_buf)
        axA.relim(); axA.autoscale_view()
        if args.show_magnitude:
            l_gmag.set_data(t_buf, gmag_buf); l_amag.set_data(t_buf, amag_buf)

    # Magnetometer uses its own time buffer (avoid length mismatch)
    if len(t_mag_buf) > 1 and (imu_plot_tick % IMU_AUTOSCALE_EVERY_N == 0):
        l_mx.set_data(t_mag_buf, mx_buf); l_my.set_data(t_mag_buf, my_buf); l_mz.set_data(t_mag_buf, mz_buf)
        axM.relim(); axM.autoscale_view()
        if args.show_magnitude and len(mmag_buf) > 1:
            l_mmag.set_data(t_mag_buf, mmag_buf)

    # 3D cube update (if enabled)
    extra = []
    if args.show_3d and shape_poly is not None:
        updated = _update_cube_artist()
        if updated is not None:
            extra.append(updated)

    return [im, l_gx, l_gy, l_gz, l_ax, l_ay, l_az, l_mx, l_my, l_mz] + extra

# -----------------------------
# Run
# -----------------------------
ani = FuncAnimation(fig, on_timer, interval=ANIM_INTERVAL_MS, blit=False, cache_frame_data=False)
print("[RUN] Close the window to exit.")
try:
    plt.show()
except KeyboardInterrupt:
    # Allow Ctrl+C exit from terminal without noisy Tk callback traceback.
    pass
finally:
    try:
        sock.close()
    except Exception:
        pass
    if csv_fp:
        csv_fp.close()
    if session_fp:
        session_fp.close()
