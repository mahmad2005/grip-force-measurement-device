"""Standalone desktop launcher for viewer.html.

This runs a local HTTP server for the viewer assets and opens the page in a
native app window via pywebview, so it does not require opening a browser tab.

Persistent layout note:
- The viewer saves floating panel layout in browser localStorage.
- pywebview defaults to private mode in many versions, so localStorage may be
  cleared when the app closes unless private_mode=False and a stable storage
  path are provided to webview.start().
"""

from __future__ import annotations

import argparse
import functools
import json
import os
import socket
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


DEFAULT_STANDALONE_PORT = 8765


def _pick_free_port(host: str) -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind((host, 0))
        return int(sock.getsockname()[1])


def _default_storage_dir() -> Path:
    """Return a stable pywebview storage directory for cookies/localStorage."""
    if os.name == "nt":
        root = os.environ.get("LOCALAPPDATA")
        if root:
            return Path(root) / "KinesiologyLabViewer" / "pywebview_storage"
        return Path.home() / "AppData" / "Local" / "KinesiologyLabViewer" / "pywebview_storage"
    return Path.home() / ".kinesiologylab_viewer" / "pywebview_storage"


class _QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, format: str, *args: object) -> None:
        return

    def do_POST(self) -> None:
        if self.path != "/save-recording":
            self.send_error(404, "Not found")
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = 0

        if length <= 0:
            self.send_error(400, "Empty body")
            return

        payload = self.rfile.read(length)
        requested_name = self.headers.get("X-Filename", "viewer_recording.webm")
        safe_name = Path(requested_name).name or "viewer_recording.webm"

        download_dir = Path.home() / "Downloads"
        target_dir = download_dir if download_dir.exists() else Path(self.directory or ".")
        target_dir.mkdir(parents=True, exist_ok=True)

        target = target_dir / safe_name
        stem = target.stem
        suffix = target.suffix
        counter = 1
        while target.exists():
            target = target_dir / f"{stem}_{counter}{suffix}"
            counter += 1

        try:
            target.write_bytes(payload)
        except OSError as exc:
            body = json.dumps({"ok": False, "error": str(exc)}).encode("utf-8")
            self.send_response(500)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        body = json.dumps({"ok": True, "file": target.name, "path": str(target)}).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Open the IMU/pressure viewer as a desktop app")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP host to bind (default: 127.0.0.1)")
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_STANDALONE_PORT,
        help=f"HTTP port to bind (default: {DEFAULT_STANDALONE_PORT}; use 0 for auto)",
    )
    parser.add_argument("--file", default="viewer.html", help="Viewer file relative to this script")
    parser.add_argument("--title", default="KinesiologyLab Viewer", help="Desktop window title")
    parser.add_argument("--width", type=int, default=1600, help="Window width")
    parser.add_argument("--height", type=int, default=950, help="Window height")
    parser.add_argument("--debug", action="store_true", help="Enable pywebview debug mode")

    # New: stable localStorage/cookie storage for persistent panel positions.
    parser.add_argument(
        "--storage-dir",
        default=None,
        help=(
            "Folder for pywebview cookies/localStorage. "
            "Default: a stable KinesiologyLabViewer folder in the user profile."
        ),
    )
    parser.add_argument(
        "--private",
        action="store_true",
        help="Run pywebview in private mode. Layout/cookies/localStorage will NOT persist.",
    )
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

    if args.port == 0:
        port = _pick_free_port(args.host)
    else:
        port = args.port

    handler_cls = functools.partial(_QuietHandler, directory=str(base_dir))
    try:
        server = ThreadingHTTPServer((args.host, port), handler_cls)
    except OSError:
        # If default fixed port is busy, fall back to an available one.
        # Note: localStorage is origin-specific, so a different port may have a
        # separate saved layout. Close the old instance or use --port 8765 for
        # the same saved layout.
        if args.port == DEFAULT_STANDALONE_PORT:
            port = _pick_free_port(args.host)
            print(
                f"Warning: port {DEFAULT_STANDALONE_PORT} is busy. "
                f"Using port {port}. Saved layout may be separate for this port."
            )
            server = ThreadingHTTPServer((args.host, port), handler_cls)
        else:
            raise

    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    rel = viewer_path.relative_to(base_dir).as_posix()
    url = f"http://{args.host}:{port}/{rel}"

    storage_dir = Path(args.storage_dir).expanduser().resolve() if args.storage_dir else _default_storage_dir()
    storage_dir.mkdir(parents=True, exist_ok=True)

    print(f"Opening: {url}")
    print(f"pywebview storage: {storage_dir}")
    print("Panel layout persistence: " + ("OFF because --private was used" if args.private else "ON"))

    try:
        webview.create_window(args.title, url=url, width=args.width, height=args.height)

        # Important for persistence:
        # private_mode=False allows cookies/localStorage to be written between runs.
        # storage_path gives WebView2/pywebview a stable place to save that data.
        try:
            webview.start(
                debug=args.debug,
                private_mode=bool(args.private),
                storage_path=str(storage_dir),
            )
        except TypeError as exc:
            print("This pywebview version did not accept private_mode/storage_path.")
            print(f"Original error: {exc}")
            print("Update pywebview with: pip install --upgrade pywebview")
            print("Starting without explicit persistent storage...")
            webview.start(debug=args.debug)
    finally:
        server.shutdown()
        server.server_close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
