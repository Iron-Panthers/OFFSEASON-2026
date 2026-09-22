"""Serves the annotated frames back out as MJPEG.

Deliberately the same endpoint shape as the renderer's own server, so anything that already points
at a camera stream works against the detector's output with only the port changed.
"""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

_BOUNDARY = "frameboundary"

# Long enough that an idle stream still sends something before a browser gives up on it.
_FRAME_WAIT_SECONDS = 2.0

_INDEX_PAGE = """<!doctype html>
<html><head><title>{name}</title>
<style>body{{margin:0;background:#111;color:#ddd;font:14px system-ui;text-align:center}}
img{{max-width:100%;height:auto;image-rendering:pixelated}}
p{{padding:8px}}</style></head>
<body><p>{name}</p><img src="/stream.mjpg" alt="stream"></body></html>
"""


class AnnotatedStream:
    """A one-camera MJPEG endpoint fed by :meth:`publish`.

    Frames are only encoded by the caller when :meth:`wanted` is true, so a detector that nobody
    is watching does not pay for JPEG encoding on every frame.
    """

    def __init__(self, name: str, port: int):
        self.name = name
        self.port = port
        self._lock = threading.Condition()
        self._jpeg: Optional[bytes] = None
        self._counter = 0
        self._viewers = 0
        self._started = time.time()
        self._server: Optional[ThreadingHTTPServer] = None

    def start(self) -> "AnnotatedStream":
        stream = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *_args):  # noqa: N802 - stdlib signature
                pass  # the supervisor already prefixes our stdout; request spam helps nobody

            def do_GET(self):  # noqa: N802 - stdlib signature
                if self.path.startswith("/stream.mjpg"):
                    stream._serve_stream(self)
                elif self.path.startswith("/snapshot.jpg"):
                    stream._serve_snapshot(self)
                elif self.path.startswith("/info.json"):
                    stream._serve_info(self)
                else:
                    stream._serve_index(self)

        # Loopback only. This is development output, not something to put on a field network.
        self._server = ThreadingHTTPServer(("127.0.0.1", self.port), Handler)
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, name="AnnotatedStream", daemon=True).start()
        return self

    def stop(self) -> None:
        with self._lock:
            self._lock.notify_all()
        if self._server is not None:
            self._server.shutdown()

    @property
    def wanted(self) -> bool:
        """True when at least one client is reading the stream."""
        return self._viewers > 0

    def publish(self, jpeg: bytes) -> None:
        with self._lock:
            self._jpeg = jpeg
            self._counter += 1
            self._lock.notify_all()

    # -- handlers ----------------------------------------------------------

    def _serve_index(self, handler: BaseHTTPRequestHandler) -> None:
        body = _INDEX_PAGE.format(name=self.name).encode("utf-8")
        handler.send_response(200)
        handler.send_header("Content-Type", "text/html; charset=utf-8")
        handler.send_header("Content-Length", str(len(body)))
        handler.end_headers()
        handler.wfile.write(body)

    def _serve_info(self, handler: BaseHTTPRequestHandler) -> None:
        with self._lock:
            body = json.dumps(
                {
                    "name": self.name,
                    "port": self.port,
                    "frames": self._counter,
                    "viewers": self._viewers,
                    "uptimeSeconds": round(time.time() - self._started, 1),
                }
            ).encode("utf-8")
        handler.send_response(200)
        handler.send_header("Content-Type", "application/json")
        handler.send_header("Content-Length", str(len(body)))
        handler.end_headers()
        handler.wfile.write(body)

    def _serve_snapshot(self, handler: BaseHTTPRequestHandler) -> None:
        with self._lock:
            jpeg = self._jpeg
        if jpeg is None:
            handler.send_response(503)
            handler.send_header("Content-Length", "0")
            handler.end_headers()
            return
        handler.send_response(200)
        handler.send_header("Content-Type", "image/jpeg")
        handler.send_header("Content-Length", str(len(jpeg)))
        handler.send_header("Cache-Control", "no-store")
        handler.end_headers()
        handler.wfile.write(jpeg)

    def _serve_stream(self, handler: BaseHTTPRequestHandler) -> None:
        handler.send_response(200)
        handler.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={_BOUNDARY}")
        handler.send_header("Cache-Control", "no-store")
        handler.end_headers()

        with self._lock:
            self._viewers += 1
        sent = -1
        try:
            while True:
                with self._lock:
                    # Wait for something newer, but time out and resend the current frame: a
                    # browser drops a multipart stream that goes completely silent, and at
                    # simulated frame rates that silence can last seconds.
                    if self._jpeg is None or self._counter == sent:
                        self._lock.wait(_FRAME_WAIT_SECONDS)
                    if self._jpeg is None:
                        continue
                    jpeg = self._jpeg
                    sent = self._counter

                handler.wfile.write(f"--{_BOUNDARY}\r\n".encode("ascii"))
                handler.wfile.write(b"Content-Type: image/jpeg\r\n")
                handler.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                handler.wfile.write(jpeg)
                handler.wfile.write(b"\r\n")
                handler.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass  # a closed tab is the normal way this ends
        finally:
            with self._lock:
                self._viewers -= 1
