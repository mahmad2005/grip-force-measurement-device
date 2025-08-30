#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
viz_full.py — resilient 32×64 grip-pressure visualizer for
SPI -> ESP32 -> UDP -> PC

Packet (per chunk, 415 bytes):
  [0]  0xAA
  [1]  0x55
  [2]  chunk_index in 0..9
  [3..412] 410-byte payload = 205 × uint16
  [413] 0x55
  [414] 0xAA

We assemble 10 chunks -> 2050 samples; first 2048 map to a 32×64 frame.
If chunks are late/missing, we DROP the partial frame after a timeout and
keep the UI responsive.
"""

import argparse
import socket
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# -----------------------------
# Config (change if needed)
# -----------------------------
FRAME_W, FRAME_H = 64, 32
FRAME_SIZE = FRAME_W * FRAME_H

CHUNKS_PER_FRAME = 10
VALUES_PER_CHUNK = 205
BYTES_PER_PACKET = 415

UDP_DRAIN_PER_TICK = 100          # how many packets to drain per UI tick
ANIM_INTERVAL_MS = 5              # UI tick period

# Timeouts to prevent "stuck"
FRAME_TIMEOUT_S = 0.20            # drop partial frame if not complete in time
STREAM_SILENCE_RESET_S = 1.5      # if no packets at all for this long, reset buffers

# Visualization scale (set USE_AUTO_CLIM=True to autoscale by frame)
VMIN, VMAX = 0, 4095              # good for 12-bit ADC
USE_AUTO_CLIM = False

# Optional geometry transforms
APPLY_FLIP_LR = False
ROTATE_K = 0                      # 0,1,2,3
APPLY_HALF_MIRROR = True          # NEW: mirror each 32-col half independently

# Endianness handling:
#   "AUTO" -> detect once from the first plausible frame, then lock
#   "LE"   -> treat payload as little-endian uint16 (STM32 default)
#   "BE"   -> treat payload as big-endian uint16
ENDIAN_MODE = "AUTO"

# -----------------------------
# CLI
# -----------------------------
parser = argparse.ArgumentParser(description="32x64 UDP visualizer (robust)")
parser.add_argument("--ip", default="0.0.0.0", help="IP to bind")
parser.add_argument("--port", type=int, default=12345, help="UDP port")
args, _ = parser.parse_known_args()

# -----------------------------
# UDP socket (non-blocking)
# -----------------------------
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((args.ip, args.port))
sock.setblocking(False)
print(f"Listening on UDP {args.ip}:{args.port}")

# -----------------------------
# State
# -----------------------------
# We store each chunk as the raw little-endian interpretation (fast),
# and swap bytes later if BE is chosen.
pending_chunks = [None] * CHUNKS_PER_FRAME  # each entry: np.ndarray('<u2', 205)
received_idxs = set()
frame_start_ts = None
last_packet_ts = time.time()

# stats
frames_ok = 0
frames_dropped = 0
bad_packets = 0
smoothed_fps = 0.0
fps_window_start = time.time()
fps_frames = 0

# Endianness mode (may be determined after first valid frame)
chosen_endian = ENDIAN_MODE  # "AUTO" | "LE" | "BE"

# -----------------------------
# Matplotlib setup
# -----------------------------
plt.rcParams["toolbar"] = "toolbar2"
fig, ax = plt.subplots(figsize=(9.5, 5.0))
mat0 = np.zeros((FRAME_H, FRAME_W), dtype=np.uint16)
im = ax.imshow(mat0, cmap="rainbow", vmin=VMIN, vmax=VMAX, aspect="auto")
cbar = fig.colorbar(im, ax=ax)
cbar.set_label("ADC / Pressure")

title = ax.set_title("32×64 Grip Pressure Visualization - 0.0 FPS")
ax.set_xlabel("X (cols)")
ax.set_ylabel("Y (rows)")
ax.set_xlim(-0.5, FRAME_W - 0.5)
ax.set_ylim(FRAME_H - 0.5, -0.5)  # put (0,0) at top-left

# --- add near the top (globals) ---
last_hash = None
unchanged_count = 0
STUCK_THRESH = 30        # frames
status_txt = ax.text(0.01, 0.02, "", color="w", transform=ax.transAxes,
                     fontsize=10, bbox=dict(facecolor="black", alpha=0.4, pad=3))


# -----------------------------
# Helpers
# -----------------------------
def reset_frame_buffers():
    global pending_chunks, received_idxs, frame_start_ts
    pending_chunks = [None] * CHUNKS_PER_FRAME
    received_idxs.clear()
    frame_start_ts = None

def try_recv_once():
    """Non-blocking recv; returns bytes or None."""
    global last_packet_ts
    try:
        data, _ = sock.recvfrom(2048)
        last_packet_ts = time.time()
        return data
    except BlockingIOError:
        return None

def parse_and_store(pkt: bytes):
    """Parse a 415B packet; if valid, store its LE-decoded values in pending_chunks."""
    global bad_packets, frame_start_ts
    if pkt is None or len(pkt) != BYTES_PER_PACKET:
        if pkt is not None:
            bad_packets += 1
        return

    if not (pkt[0] == 0xAA and pkt[1] == 0x55 and pkt[-2] == 0x55 and pkt[-1] == 0xAA):
        bad_packets += 1
        return

    idx = pkt[2]
    if idx >= CHUNKS_PER_FRAME:
        bad_packets += 1
        return

    payload = pkt[3:-2]  # 410 bytes
    if len(payload) != 410:
        bad_packets += 1
        return

    # Decode once as little-endian (fast). We'll byteswap later if needed.
    vals_le = np.frombuffer(payload, dtype="<u2", count=VALUES_PER_CHUNK)
    if vals_le.size != VALUES_PER_CHUNK:
        bad_packets += 1
        return

    # start-of-frame timestamp
    if not received_idxs:
        frame_start_ts = time.time()

    # only accept the first copy of each index for this frame
    if idx not in received_idxs:
        pending_chunks[idx] = vals_le
        received_idxs.add(idx)

def choose_endianness(all_le: np.ndarray) -> str:
    """Pick endian mode based on plausible value ranges; returns 'LE' or 'BE'."""
    # If values fit 0..4095 as-is, assume LE.
    le_max = int(all_le.max())
    if le_max <= 4095:
        return "LE"
    # If byteswapped values fit 0..4095, assume BE.
    be_max = int(all_le.byteswap().max())
    if be_max <= 4095:
        return "BE"
    # Otherwise default to LE (you can change to 'BE' if your system is big-endian)
    return "LE"

def build_and_show_frame():
    """Assemble the frame, apply endian/geometry, and update the plot & FPS."""
    global frames_ok, smoothed_fps, fps_frames, fps_window_start, chosen_endian

    # Concatenate chunks in index order 0..9 (2050 values); use first 2048
    all_le = np.concatenate(pending_chunks)[:FRAME_SIZE]

    # Lock endianness the first time we see a plausible frame (AUTO mode)
    if chosen_endian == "AUTO":
        chosen_endian = choose_endianness(all_le)

    if chosen_endian == "BE":
        all_vals = all_le.byteswap()
    else:
        all_vals = all_le

    mat = all_vals.reshape(FRAME_H, FRAME_W)

    # NEW: Mirror each 32-column half independently
    if APPLY_HALF_MIRROR:
        mid = FRAME_W // 2  # 64 -> 32
        left  = mat[:, :mid][:, ::-1]   # flip columns 0..31
        right = mat[:, mid:][:, ::-1]   # flip columns 32..63
        mat = np.concatenate((left, right), axis=1)

    # Optional transforms
    if APPLY_FLIP_LR:
        mat = np.fliplr(mat)
    if ROTATE_K:
        mat = np.rot90(mat, k=ROTATE_K)

    # Color scaling
    if USE_AUTO_CLIM:
        lo, hi = np.percentile(mat, [5, 95])
        if hi <= lo:
            hi = lo + 1
        im.set_clim(lo, hi)
    else:
        im.set_clim(VMIN, VMAX)

    im.set_data(mat)

    # ---- STUCK FRAME DETECTOR ----
    # Fast 32-bit hash of the frame; good enough for stuck detection
    h = int(np.uint32(mat.sum() * 2654435761 & 0xFFFFFFFF))
    global last_hash, unchanged_count
    if last_hash is None or h != last_hash:
        unchanged_count = 0
        last_hash = h
    else:
        unchanged_count += 1

    if unchanged_count >= STUCK_THRESH:
        status_txt.set_text(f"⚠ STUCK {unchanged_count} frames")
    else:
        status_txt.set_text("")

    # FPS (smoothed)
    fps_frames += 1
    now = time.time()
    dt = now - fps_window_start
    if dt >= 0.5:
        inst = fps_frames / dt
        smoothed_fps = inst if smoothed_fps == 0.0 else (0.7 * smoothed_fps + 0.3 * inst)
        fps_frames = 0
        fps_window_start = now

    frames_ok += 1
    title.set_text(f"32×64 Grip Pressure Visualization - {smoothed_fps:.1f} FPS")

# -----------------------------
# Animation tick
# -----------------------------
def on_timer(_):
    """Drain UDP, assemble frames with timeouts so we never get stuck."""
    global frames_dropped

    # 1) Drain a bunch of UDP packets so we keep up with the stream
    for _ in range(UDP_DRAIN_PER_TICK):
        pkt = try_recv_once()
        if pkt is None:
            break
        parse_and_store(pkt)

        # If we just finished a frame, draw it and reset state immediately
        if len(received_idxs) == CHUNKS_PER_FRAME and all(ch is not None for ch in pending_chunks):
            build_and_show_frame()
            reset_frame_buffers()
            # keep draining this tick (do NOT return early)

    # 2) Handle partial-frame timeout (drop and restart)
    if received_idxs and frame_start_ts is not None:
        if (time.time() - frame_start_ts) > FRAME_TIMEOUT_S:
            frames_dropped += 1
            reset_frame_buffers()

    # 3) Handle long stream silence (reset everything so UI never hangs)
    if (time.time() - last_packet_ts) > STREAM_SILENCE_RESET_S:
        reset_frame_buffers()

    return [im]

# -----------------------------
# Run
# -----------------------------
ani = FuncAnimation(fig, on_timer, interval=ANIM_INTERVAL_MS, blit=False)
plt.tight_layout()
plt.show()
sock.close()
