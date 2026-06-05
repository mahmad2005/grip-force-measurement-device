#!/usr/bin/env python3
"""
viewer_tuner_server.py

Run this from the folder that contains:
  - viewer_tuner.html
  - viewer_data.json

Then open:
  http://localhost:8000/viewer_tuner.html

The normal Python static server can show the page, but it cannot save files.
This server adds one POST endpoint: /save-json
It saves the adjusted JSON back to viewer_data.json and creates a timestamped backup first.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from datetime import datetime
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse


class TunerHandler(SimpleHTTPRequestHandler):
    save_filename = "viewer_data.json"
    max_upload_bytes = 100 * 1024 * 1024  # 100 MB

    def end_json(self, status: int, payload: dict) -> None:
        body = json.dumps(payload, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path != "/save-json":
            self.end_json(404, {"ok": False, "error": "Unknown endpoint"})
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self.end_json(400, {"ok": False, "error": "Invalid Content-Length"})
            return

        if length <= 0:
            self.end_json(400, {"ok": False, "error": "Empty request body"})
            return
        if length > self.max_upload_bytes:
            self.end_json(413, {"ok": False, "error": "JSON too large"})
            return

        raw = self.rfile.read(length)
        try:
            data = json.loads(raw.decode("utf-8"))
        except Exception as exc:
            self.end_json(400, {"ok": False, "error": f"Invalid JSON: {exc}"})
            return

        output_path = Path.cwd() / self.save_filename
        try:
            if output_path.exists():
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                backup_path = output_path.with_name(f"{output_path.stem}.backup_{stamp}{output_path.suffix}")
                backup_path.write_bytes(output_path.read_bytes())
            else:
                backup_path = None

            output_path.write_text(json.dumps(data, indent=2), encoding="utf-8")
            self.end_json(
                200,
                {
                    "ok": True,
                    "file": str(output_path.name),
                    "backup": str(backup_path.name) if backup_path else None,
                },
            )
        except Exception as exc:
            self.end_json(500, {"ok": False, "error": f"Save failed: {exc}"})


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve viewer_tuner.html and allow saving adjusted viewer_data.json")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--file", default="viewer_data.json", help="JSON filename to overwrite when Save is pressed")
    args = parser.parse_args()

    TunerHandler.save_filename = args.file

    server = ThreadingHTTPServer((args.host, args.port), TunerHandler)
    print(f"Serving current folder: {Path.cwd()}")
    print(f"Open: http://{args.host}:{args.port}/viewer_tuner.html")
    print(f"Save button writes: {Path.cwd() / args.file}")
    print("Press Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
