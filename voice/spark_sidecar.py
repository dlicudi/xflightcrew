#!/usr/bin/env python3
"""Warm Spark-TTS sidecar for x-copilot.

Loads the voice-clone model once and serves it over HTTP: POST readback text to
`/synth`, get back `audio/wav` in the cloned voice. The Rust app's
`tts_backend = "sidecar"` talks to this. Renders sentence-by-sentence (Spark
drifts on long single generations) and stitches the result.

Voices live in voice/voices/<name>/ref.{wav,txt}. The active voice is resolved
**per request**, so `voice/voices.py use <name>` switches the voice live without
restarting anything. Resolution order: voices/.active file → ROGER_SIDECAR_VOICE
env → "roger" (with a fallback to the legacy voice/roger_voice_ref.{wav,txt}).

The Rust app auto-spawns this (XCOPILOT_TTS=sidecar, the default) and supervises
it, but you can also run it by hand:
    /tmp/mlxtts/bin/python voice/spark_sidecar.py
then start the app with XCOPILOT_TTS_SIDECAR_AUTOSPAWN=false.

When ROGER_SIDECAR_PARENT_WATCH=1 (set by the Rust supervisor), a daemon thread
watches stdin and exits the process on EOF — so when the parent dies (even on a
hard Ctrl-C that skips Rust's Drop), the closed pipe takes this sidecar down with
it and no orphan model server is left running. Manual runs don't set it, so their
stdin is left alone.
"""
import os, re, glob, io, wave, sys, threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from mlx_audio.tts.utils import load_model
from mlx_audio.tts.generate import generate_audio


def _watch_parent():
    """Exit when stdin reaches EOF — i.e. the supervising parent's pipe closed."""
    try:
        sys.stdin.read()
    except Exception:
        pass
    os._exit(0)


if os.environ.get("ROGER_SIDECAR_PARENT_WATCH"):
    threading.Thread(target=_watch_parent, daemon=True).start()

MODEL = "mlx-community/Spark-TTS-0.5B-bf16"
HERE = os.path.dirname(os.path.abspath(__file__))
VOICES_DIR = os.path.join(HERE, "voices")
TMP = "/tmp/roger_sidecar_seg"
HOST = os.environ.get("ROGER_SIDECAR_HOST", "127.0.0.1")
PORT = int(os.environ.get("ROGER_SIDECAR_PORT", "8765"))
# Sampling temperature: higher = livelier/more varied prosody, lower = flatter.
# 0.6 is the known-good value — it preserved accent/identity on a long reference.
TEMP = float(os.environ.get("ROGER_SIDECAR_TEMP", "0.6"))


def active_voice() -> str:
    """Which voice to speak in: the `use`-set .active file wins, then the env
    default, then "roger". Resolved per request so switching is live."""
    sel = os.path.join(VOICES_DIR, ".active")
    if os.path.exists(sel):
        name = open(sel).read().strip()
        if name:
            return name
    return os.environ.get("ROGER_SIDECAR_VOICE", "").strip() or "roger"


def voice_ref():
    """(ref_wav, ref_text, name) for the active voice, with a fallback to the
    legacy single-file reference if the library entry isn't there yet."""
    name = active_voice()
    wav = os.path.join(VOICES_DIR, name, "ref.wav")
    txt = os.path.join(VOICES_DIR, name, "ref.txt")
    if not os.path.exists(wav):
        legacy = os.path.join(HERE, "roger_voice_ref.wav")
        if os.path.exists(legacy):
            return legacy, os.path.join(HERE, "roger_voice_ref.txt"), name
    return wav, txt, name


print(f"Loading Spark voice model (one-time, temp={TEMP})…", flush=True)
MODEL_OBJ = load_model(MODEL)
print("ready.", flush=True)


def synth(text: str, ref_wav: str = None, ref_text: str = None, temp: float = None) -> bytes:
    """Render text sentence-by-sentence; return one stitched 16-bit mono WAV.
    `ref_wav`/`ref_text`/`temp` override the active voice and TEMP for one call —
    used by `voices.py say --compare` to A/B voices and temperatures."""
    if ref_wav is None:
        ref_wav, ref_txt_path, _ = voice_ref()
        ref_text = open(ref_txt_path).read().strip()
    if temp is None:
        temp = TEMP
    segs = []
    for s in re.split(r"(?<=[.!?])\s+", text.strip()):
        s = s.strip()
        if not s:
            continue
        for f in glob.glob(TMP + "*.wav"):
            os.remove(f)
        generate_audio(text=s, model=MODEL_OBJ, ref_audio=ref_wav, ref_text=ref_text,
                       temperature=temp, file_prefix=TMP, audio_format="wav",
                       join_audio=True, save=True, verbose=False)
        files = sorted(glob.glob(TMP + "*.wav"))
        if files:
            w = wave.open(files[0])
            segs.append((w.getframerate(), w.readframes(w.getnframes())))
    if not segs:
        return b""
    sr = segs[0][0]
    buf = io.BytesIO()
    out = wave.open(buf, "w")
    out.setnchannels(1); out.setsampwidth(2); out.setframerate(sr)
    gap = b"\x00\x00" * int(0.18 * sr)
    for i, (_, frames) in enumerate(segs):
        out.writeframes(frames)
        if i < len(segs) - 1:
            out.writeframes(gap)
    out.close()
    return buf.getvalue()


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0"))
        text = self.rfile.read(n).decode("utf-8", "ignore")
        try:
            wav = synth(text)
            self.send_response(200)
            self.send_header("Content-Type", "audio/wav")
            self.send_header("Content-Length", str(len(wav)))
            self.end_headers()
            self.wfile.write(wav)
        except Exception as e:  # keep the server alive on a bad request
            msg = str(e).encode()
            self.send_response(500)
            self.send_header("Content-Length", str(len(msg)))
            self.end_headers()
            self.wfile.write(msg)

    def log_message(self, *a):
        pass


if __name__ == "__main__":
    print(f"listening on http://{HOST}:{PORT}/synth", flush=True)
    # Single-threaded: MLX's Metal stream is thread-local, so synthesis must run
    # on the thread that loaded the model. Readbacks are serial anyway.
    HTTPServer((HOST, PORT), Handler).serve_forever()
