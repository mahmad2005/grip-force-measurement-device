"""Standalone desktop launcher for viewer.html.

This runs a local HTTP server for the viewer assets and opens the page in a
native app window via pywebview, so it does not require opening a browser tab.
"""

from __future__ import annotations

import argparse
import functools
import socket
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


def _pick_free_port(host: str) -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind((host, 0))
        return int(sock.getsockname()[1])


class _QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, format: str, *args: object) -> None:
        return


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Open the IMU/pressure viewer as a desktop app")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP host to bind (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=0, help="HTTP port to bind (default: auto)")
    parser.add_argument("--file", default="viewer.html", help="Viewer file relative to this script")
    parser.add_argument("--title", default="KinesiologyLab Viewer", help="Desktop window title")
    parser.add_argument("--width", type=int, default=1600, help="Window width")
    parser.add_argument("--height", type=int, default=950, help="Window height")
    parser.add_argument("--debug", action="store_true", help="Enable pywebview debug mode")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    base_dir = Path(__file__).resolve().parent
    viewer_path = (base_dir / args.file).resolve()

    if not viewer_path.exists():
        print(f"Viewer file not found: {viewer_path}")
        return 1

    try:
        import webview  # type: ignore
    except Exception:
        print("pywebview is required for standalone mode.")
        print("Install it with: pip install pywebview")
        return 1

    port = args.port if args.port > 0 else _pick_free_port(args.host)
    handler_cls = functools.partial(_QuietHandler, directory=str(base_dir))
    server = ThreadingHTTPServer((args.host, port), handler_cls)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    rel = viewer_path.relative_to(base_dir).as_posix()
    url = f"http://{args.host}:{port}/{rel}"

    try:
        webview.create_window(args.title, url=url, width=args.width, height=args.height)
        webview.start(debug=args.debug)
    finally:
        server.shutdown()
        server.server_close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
