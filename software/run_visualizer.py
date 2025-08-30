#!/usr/bin/env python3
"""
Launch the 32×64 UDP visualizer using settings from viz_config.json.

Usage (Windows cmd):
  python software\run_visualizer.py
"""
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SW_DIR = os.path.join(ROOT, "software")
CFG = os.path.join(SW_DIR, "viz_config.json")
SCRIPT = os.path.join(SW_DIR, "viz_full_4.py")

def main():
    ip = "0.0.0.0"
    port = 12345
    if os.path.exists(CFG):
        with open(CFG, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        ip = str(cfg.get("ip", ip))
        port = int(cfg.get("port", port))

    args = [sys.executable, SCRIPT, "--ip", ip, "--port", str(port)]
    env = os.environ.copy()
    # Prefer unbuffered for quicker prints
    env.setdefault("PYTHONUNBUFFERED", "1")
    try:
        proc = subprocess.Popen(args, cwd=SW_DIR, env=env)
        proc.wait()
        return proc.returncode
    except KeyboardInterrupt:
        return 0

if __name__ == "__main__":
    sys.exit(main())
