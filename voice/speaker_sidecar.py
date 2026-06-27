#!/usr/bin/env python3
"""Warm speaker-embedding sidecar for x-copilot.

Loads the Resemblyzer voice encoder once and serves embeddings over HTTP, so the
Rust app can recognise the pilot's voice (and ignore X-Plane ATC / ambient
speech) without paying the torch-import cost per utterance.

    POST /embed   body = raw little-endian f32 PCM, mono, 16 kHz
                  -> {"embedding": [...256 floats, L2-normalized...]}
    GET  /health  -> 200 "ok"

The Rust app stores the enrolled voiceprint and computes cosine similarity
itself; this service is stateless. It prints "ready." on stdout once the model is
loaded (the Rust supervisor waits for that), and — when ROGER_SIDECAR_PARENT_WATCH
is set — exits when the parent's stdin pipe closes, so no orphan is left behind.

Run by hand:
    pip install resemblyzer
    python3 voice/speaker_sidecar.py
"""

import os
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np
from resemblyzer import VoiceEncoder, preprocess_wav

HOST = os.environ.get("XCOPILOT_SPEAKER_HOST", "127.0.0.1")
PORT = int(os.environ.get("XCOPILOT_SPEAKER_PORT", "8770"))
SAMPLE_RATE = 16000


def _watch_parent() -> None:
    """Exit when stdin reaches EOF — i.e. the supervising parent's pipe closed."""
    try:
        sys.stdin.read()
    except Exception:
        pass
    os._exit(0)


ENCODER = VoiceEncoder()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        pass  # quiet; the Rust side logs what it needs

    def do_GET(self):
        if self.path == "/health":
            self._send(200, b"ok")
        else:
            self._send(404, b"not found")

    def do_POST(self):
        if self.path != "/embed":
            self._send(404, b"not found")
            return
        n = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(n) if n else b""
        try:
            samples = np.frombuffer(raw, dtype="<f4").astype(np.float32)
            wav = preprocess_wav(samples, source_sr=SAMPLE_RATE)
            emb = ENCODER.embed_utterance(wav)
            body = b'{"embedding":[' + b",".join(f"{x:.6f}".encode() for x in emb) + b"]}"
            self._send(200, body, "application/json")
        except Exception as e:  # noqa: BLE001 - report and 500 rather than crash
            self._send(500, f"embed failed: {e}".encode())

    def _send(self, code: int, body: bytes, ctype: str = "text/plain"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main() -> None:
    if os.environ.get("ROGER_SIDECAR_PARENT_WATCH"):
        threading.Thread(target=_watch_parent, daemon=True).start()
    server = HTTPServer((HOST, PORT), Handler)
    print("ready.", flush=True)  # the Rust supervisor blocks on this line
    server.serve_forever()


if __name__ == "__main__":
    main()
