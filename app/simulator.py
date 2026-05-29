#!/usr/bin/env python3
"""
Simulator for 32x64 Grip Pressure + IMU UDP Device
- Simulates moving hand pressure patterns
- Simulates IMU (gyro, accel, mag) data
- Sends UDP packets compatible with the visualization software
- Adjustable FPS and interactive controls
"""

import socket
import struct
import numpy as np
import time
import argparse
import math
import sys

try:
    import msvcrt  # Windows non-blocking keyboard input
except ImportError:
    msvcrt = None

# --- Constants (must match visualizer) ---
FRAME_W, FRAME_H = 64, 32
FRAME_SIZE = FRAME_W * FRAME_H
CHUNKS_PER_FRAME = 10
VALUES_PER_CHUNK = 205
BYTES_PER_PACKET = 415

# IMU packet formats
HDR_IM = b'\xAA\x55IM'  # gyro+accel+mag
LEN_IM = 4 + (9 * 2)     # 22 bytes
GYRO_SF = 131.0
ACC_SF = 16384.0
MAG_SF_uT = 0.15

# Sensor ranges
PRESSURE_MIN, PRESSURE_MAX = 0, 4095
GYRO_RANGE = 250  # deg/s
ACC_RANGE = 2     # g
MAG_RANGE = 120   # uT (typical)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))

# --- Simulator ---
def make_hand_pattern(cx, cy, radius=7, strength=3500):
    """Return a 32x64 array with a circular pressure blob centered at (cx, cy)."""
    y, x = np.ogrid[:FRAME_H, :FRAME_W]
    mask = (x - cx)**2 + (y - cy)**2 <= radius**2
    arr = np.zeros((FRAME_H, FRAME_W), dtype=np.uint16)
    arr[mask] = strength
    # Add some random noise
    arr += np.random.randint(0, 100, arr.shape, dtype=np.uint16)
    arr = np.clip(arr, PRESSURE_MIN, PRESSURE_MAX)
    return arr

def simulate_imu(t):
    """Return (gx, gy, gz, ax, ay, az, mx, my, mz) as floats."""
    # Simulate smooth rotation and tilt
    gx = 50 * math.sin(t * 0.7)
    gy = 30 * math.cos(t * 0.5)
    gz = 20 * math.sin(t * 0.3)
    ax = math.sin(t * 0.2)
    ay = math.cos(t * 0.2)
    az = 1.0 + 0.05 * math.sin(t * 0.1)  # gravity
    mx = 40 * math.cos(t * 0.1)
    my = 40 * math.sin(t * 0.1)
    mz = 30 * math.cos(t * 0.05)
    return gx, gy, gz, ax, ay, az, mx, my, mz

def pack_pressure_packets(mat, frame_idx=0):
    """Yield 10 UDP packets for the 32x64 pressure frame."""
    flat = mat.flatten()
    for i in range(CHUNKS_PER_FRAME):
        chunk = flat[i*VALUES_PER_CHUNK:(i+1)*VALUES_PER_CHUNK]
        payload = chunk.astype('<u2').tobytes()
        # Pad to 410 bytes if needed
        if len(payload) < 410:
            payload += b'\x00' * (410 - len(payload))
        pkt = bytearray()
        pkt += b'\xAA\x55'
        pkt.append(i)
        pkt += payload
        pkt += b'\x55\xAA'
        yield pkt

def pack_imu_packet(gx, gy, gz, ax, ay, az, mx, my, mz):
    """Pack IMU data as a 22-byte packet (big-endian)."""
    vals = [
        int(clamp(gx * GYRO_SF, -32768, 32767)),
        int(clamp(gy * GYRO_SF, -32768, 32767)),
        int(clamp(gz * GYRO_SF, -32768, 32767)),
        int(clamp(ax * ACC_SF, -32768, 32767)),
        int(clamp(ay * ACC_SF, -32768, 32767)),
        int(clamp(az * ACC_SF, -32768, 32767)),
        int(clamp(mx / MAG_SF_uT, -32768, 32767)),
        int(clamp(my / MAG_SF_uT, -32768, 32767)),
        int(clamp(mz / MAG_SF_uT, -32768, 32767)),
    ]
    pkt = struct.pack('>4s9h', HDR_IM, *vals)
    return pkt


def print_controls():
    print("\n[SIM CONTROLS]")
    print("  q: quit")
    print("  h: print this help")
    print("  m: toggle IMU mode (AUTO <-> MANUAL)")
    print("  n: toggle pressure movement (AUTO <-> MANUAL)")
    print("  SPACE: reset manual IMU values")
    print("  +/-: increase/decrease pressure blob strength")
    print("  [ ]: decrease/increase pressure blob radius")
    print("  Arrow keys: move pressure blob in MANUAL pressure mode")
    print("  Gyro (deg/s): i/k=gx +/- , o/l=gy +/- , p/;=gz +/-")
    print("  Accel (g):    w/s=ax +/- , e/d=ay +/- , r/f=az +/-")
    print("  Mag (uT):     t/g=mx +/- , y/Y=my +/- , u/j=mz +/-")
    print("  Step sizes:   z/x gyro step -/+ , c/v accel step -/+ , b/B mag step -/+\n")


def handle_keyboard(state):
    """Handle non-blocking keyboard input on Windows. Returns False to quit."""
    if msvcrt is None:
        return True

    while msvcrt.kbhit():
        ch = msvcrt.getch()

        # Arrow keys are a two-byte sequence: 0xE0 then code
        if ch in (b'\x00', b'\xe0'):
            code = msvcrt.getch()
            if state["pressure_manual"]:
                if code == b'H':
                    state["cy"] = clamp(state["cy"] - 1, 0, FRAME_H - 1)
                elif code == b'P':
                    state["cy"] = clamp(state["cy"] + 1, 0, FRAME_H - 1)
                elif code == b'K':
                    state["cx"] = clamp(state["cx"] - 1, 0, FRAME_W - 1)
                elif code == b'M':
                    state["cx"] = clamp(state["cx"] + 1, 0, FRAME_W - 1)
            continue

        key = ch.decode(errors="ignore")
        if not key:
            continue

        # Global controls
        if key in ('q', 'Q'):
            return False
        if key in ('h', 'H'):
            print_controls()
        if key in ('m', 'M'):
            state["imu_manual"] = not state["imu_manual"]
            print(f"[SIM] IMU mode: {'MANUAL' if state['imu_manual'] else 'AUTO'}")
        if key in ('n', 'N'):
            state["pressure_manual"] = not state["pressure_manual"]
            print(f"[SIM] Pressure mode: {'MANUAL' if state['pressure_manual'] else 'AUTO'}")
        if key == ' ':
            state["gx"] = state["gy"] = state["gz"] = 0.0
            state["ax"], state["ay"], state["az"] = 0.0, 0.0, 1.0
            state["mx"], state["my"], state["mz"] = 40.0, 0.0, 30.0
            print("[SIM] Manual IMU reset.")

        # Pressure shape controls
        if key == '+':
            state["strength"] = clamp(state["strength"] + 100, PRESSURE_MIN, PRESSURE_MAX)
        elif key == '-':
            state["strength"] = clamp(state["strength"] - 100, PRESSURE_MIN, PRESSURE_MAX)
        elif key == '[':
            state["radius"] = clamp(state["radius"] - 1, 1, 20)
        elif key == ']':
            state["radius"] = clamp(state["radius"] + 1, 1, 20)

        # Step tuning controls
        if key in ('z', 'Z'):
            state["gyro_step"] = max(1.0, state["gyro_step"] - 1.0)
        elif key in ('x', 'X'):
            state["gyro_step"] = min(50.0, state["gyro_step"] + 1.0)
        elif key in ('c', 'C'):
            state["acc_step"] = max(0.01, state["acc_step"] - 0.01)
        elif key in ('v', 'V'):
            state["acc_step"] = min(0.50, state["acc_step"] + 0.01)
        elif key == 'b':
            state["mag_step"] = max(0.5, state["mag_step"] - 0.5)
        elif key == 'B':
            state["mag_step"] = min(20.0, state["mag_step"] + 0.5)

        # Gyro controls
        if key == 'i':
            state["gx"] = clamp(state["gx"] + state["gyro_step"], -GYRO_RANGE, GYRO_RANGE)
        elif key == 'k':
            state["gx"] = clamp(state["gx"] - state["gyro_step"], -GYRO_RANGE, GYRO_RANGE)
        elif key == 'o':
            state["gy"] = clamp(state["gy"] + state["gyro_step"], -GYRO_RANGE, GYRO_RANGE)
        elif key == 'l':
            state["gy"] = clamp(state["gy"] - state["gyro_step"], -GYRO_RANGE, GYRO_RANGE)
        elif key == 'p':
            state["gz"] = clamp(state["gz"] + state["gyro_step"], -GYRO_RANGE, GYRO_RANGE)
        elif key == ';':
            state["gz"] = clamp(state["gz"] - state["gyro_step"], -GYRO_RANGE, GYRO_RANGE)

        # Accel controls
        if key == 'w':
            state["ax"] = clamp(state["ax"] + state["acc_step"], -ACC_RANGE, ACC_RANGE)
        elif key == 's':
            state["ax"] = clamp(state["ax"] - state["acc_step"], -ACC_RANGE, ACC_RANGE)
        elif key == 'e':
            state["ay"] = clamp(state["ay"] + state["acc_step"], -ACC_RANGE, ACC_RANGE)
        elif key == 'd':
            state["ay"] = clamp(state["ay"] - state["acc_step"], -ACC_RANGE, ACC_RANGE)
        elif key == 'r':
            state["az"] = clamp(state["az"] + state["acc_step"], -ACC_RANGE, ACC_RANGE)
        elif key == 'f':
            state["az"] = clamp(state["az"] - state["acc_step"], -ACC_RANGE, ACC_RANGE)

        # Magnetometer controls
        if key == 't':
            state["mx"] = clamp(state["mx"] + state["mag_step"], -MAG_RANGE, MAG_RANGE)
        elif key == 'g':
            state["mx"] = clamp(state["mx"] - state["mag_step"], -MAG_RANGE, MAG_RANGE)
        elif key == 'y':
            state["my"] = clamp(state["my"] + state["mag_step"], -MAG_RANGE, MAG_RANGE)
        elif key == 'Y':
            state["my"] = clamp(state["my"] - state["mag_step"], -MAG_RANGE, MAG_RANGE)
        elif key == 'u':
            state["mz"] = clamp(state["mz"] + state["mag_step"], -MAG_RANGE, MAG_RANGE)
        elif key == 'j':
            state["mz"] = clamp(state["mz"] - state["mag_step"], -MAG_RANGE, MAG_RANGE)

    return True

def main():
    parser = argparse.ArgumentParser(description='Simulate 32x64 grip+IMU UDP device')
    parser.add_argument('--ip', default='127.0.0.1', help='Destination IP (visualizer host)')
    parser.add_argument('--port', type=int, default=12345, help='Destination UDP port')
    parser.add_argument('--fps', type=float, default=60, help='Frames per second')
    parser.add_argument('--manual-imu', action='store_true', help='Start with manual IMU control enabled')
    parser.add_argument('--manual-pressure', action='store_true', help='Start with manual pressure-position control enabled')
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.ip, args.port)

    state = {
        "cx": FRAME_W // 2,
        "cy": FRAME_H // 2,
        "radius": 7,
        "strength": 3500,
        "imu_manual": args.manual_imu,
        "pressure_manual": args.manual_pressure,
        "gyro_step": 5.0,
        "acc_step": 0.05,
        "mag_step": 2.0,
        "gx": 0.0,
        "gy": 0.0,
        "gz": 0.0,
        "ax": 0.0,
        "ay": 0.0,
        "az": 1.0,
        "mx": 40.0,
        "my": 0.0,
        "mz": 30.0,
    }
    t0 = time.time()
    frame_idx = 0
    status_last_print = 0.0

    print(f"[SIM] Sending to {dest} at {args.fps} FPS. Press Ctrl+C or 'q' to stop.")
    if msvcrt is None:
        print("[SIM] Keyboard controls unavailable on this platform. Running in AUTO mode behavior.")
    else:
        print_controls()

    try:
        while True:
            if not handle_keyboard(state):
                break

            t = time.time() - t0

            # Auto pressure movement if not in manual pressure mode
            if not state["pressure_manual"]:
                state["cx"] = int(FRAME_W // 2 + (FRAME_W // 3) * math.cos(t * 0.5))
                state["cy"] = int(FRAME_H // 2 + (FRAME_H // 3) * math.sin(t * 0.7))

            mat = make_hand_pattern(
                state["cx"],
                state["cy"],
                radius=state["radius"],
                strength=int(state["strength"]),
            )
            for pkt in pack_pressure_packets(mat, frame_idx):
                sock.sendto(pkt, dest)

            if state["imu_manual"]:
                gx, gy, gz = state["gx"], state["gy"], state["gz"]
                ax, ay, az = state["ax"], state["ay"], state["az"]
                mx, my, mz = state["mx"], state["my"], state["mz"]
            else:
                gx, gy, gz, ax, ay, az, mx, my, mz = simulate_imu(t)

            imu_pkt = pack_imu_packet(gx, gy, gz, ax, ay, az, mx, my, mz)
            sock.sendto(imu_pkt, dest)
            frame_idx += 1

            # Lightweight periodic status
            now = time.time()
            if now - status_last_print >= 1.0:
                status_last_print = now
                mode_txt = f"IMU={'MANUAL' if state['imu_manual'] else 'AUTO'} PRESS={'MANUAL' if state['pressure_manual'] else 'AUTO'}"
                print(
                    f"[SIM] {mode_txt} | g=({gx:6.1f},{gy:6.1f},{gz:6.1f}) dps "
                    f"a=({ax:5.2f},{ay:5.2f},{az:5.2f}) g "
                    f"m=({mx:6.1f},{my:6.1f},{mz:6.1f}) uT"
                )

            time.sleep(1.0 / args.fps)
    except KeyboardInterrupt:
        pass

    print("\n[SIM] Stopped.")

if __name__ == '__main__':
    main()
