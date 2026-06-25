#!/usr/bin/env python3
"""Manage x-copilot's cloned voices — list, switch, and add (mic or file).

Voices live in voice/voices/<name>/ref.{wav,txt}. The sidecar resolves the active
voice per request, so switching takes effect immediately — no restart.

  list                         show voices, mark the active one
  use <name>                   switch the active voice (live)
  say ["text"]                 speak a line in the active voice (preview)
  say "text" --temps 0.5,0.7   A/B the same line across temperatures
  say "text" --voices a,b      A/B the same line across voices
  add <name> --mic [--seconds N]      record a new voice from the microphone
  add <name> --file PATH [--text ...] add a voice from an existing audio file
  update <name> ...            same as add, but overwrite an existing voice
  remove <name>                delete a voice

All commands are run as:  python3 voice/voices.py <command>   (from the repo
root). Use the mlx-audio venv python for `add --mic` (it needs sounddevice).

Long references cause two problems — flat/droning tone AND reference content
leaking into readbacks — so add/update cap the clip to --max-seconds (default 20;
0 disables). The saved text is always re-derived from the final trimmed audio.

Voice-clone notes: Spark is *zero-shot* — it uses the reference as a one-shot
style+timbre prompt, it does NOT train on it. Short and energetic beats long and
flat: aim for ~10-20s of clean, lively, representative speech. A longer clip just
averages out your prosody (any tired stretch drags the whole voice down).

The reference TEXT must match the reference AUDIO. For --mic we save the exact
script you're prompted to read. For --file we use --text if given, else
auto-transcribe with the app's whisper (`x-copilot transcribe`).

Just use `python3` for everything — commands that need the model (say) or the mic
(add --mic) auto-switch to the mlx-audio venv (XCOPILOT_TTS_SIDECAR_PYTHON, else
/tmp/mlxtts/bin/python or .venv). `say` skips that entirely when the app's sidecar
is already running.
"""
import os, sys, glob, json, wave, time, shutil, argparse, subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
VOICES_DIR = os.path.join(HERE, "voices")
ACTIVE_FILE = os.path.join(VOICES_DIR, ".active")
SR = 44100  # mono; Spark resamples internally.

# Energetic, representative delivery for --mic. Deliberately NOT "calm and
# steady" — the clone mirrors exactly this prosody and energy.
MIC_SCRIPT = (
    "Roger that, let's fly! Flaps ten, trims set, controls free and correct. "
    "We're cleared for takeoff, runway two seven — power's up, airspeed alive, "
    "rotate! Positive rate, gear up. Climbing to flight level two four zero, "
    "heading three one zero. Throttle back to cruise, fuel balanced, all in "
    "the green. Field in sight — flaps full, gear down and locked, cleared to "
    "land. Easy on the flare, hold it off… and we're down. Nice one!"
)


def voices() -> list:
    if not os.path.isdir(VOICES_DIR):
        return []
    return sorted(
        d for d in os.listdir(VOICES_DIR)
        if os.path.isfile(os.path.join(VOICES_DIR, d, "ref.wav"))
    )


def active() -> str:
    if os.path.exists(ACTIVE_FILE):
        name = open(ACTIVE_FILE).read().strip()
        if name:
            return name
    return os.environ.get("XCOPILOT_VOICE", "").strip() or "roger"


def cmd_list(_args):
    cur = active()
    vs = voices()
    if not vs:
        print("no voices yet — add one with:  python3 voice/voices.py add <name> --mic")
        return
    print("voices (• = active):")
    for v in vs:
        wav = os.path.join(VOICES_DIR, v, "ref.wav")
        secs = _wav_seconds(wav)
        mark = "•" if v == cur else " "
        print(f"  {mark} {v:<16} {secs:4.1f}s")
    if cur not in vs:
        print(f"\n  (active '{cur}' has no library entry; sidecar falls back to legacy ref)")


PREVIEW_LINE = ("Roger, flaps ten. Gear up, positive rate. "
                "Climbing to flight level two four zero, all in the green.")


def cmd_say(args):
    """Synthesize a line and play it — a preview. Plain: the active voice, via the
    running sidecar (instant) or a one-shot model load. With --temps and/or
    --voices: A/B — render the same line across each temperature × voice and play
    them back to back so you can pick by ear."""
    text = args.text or PREVIEW_LINE
    temps = [float(t) for t in args.temps.split(",")] if args.temps else [None]
    vnames = [v.strip() for v in args.voices.split(",")] if args.voices else [active()]

    # Plain single preview: prefer the warm sidecar.
    if not args.temps and not args.voices:
        wav = _sidecar_say(text)
        if wav is None:
            _ensure_venv("mlx_audio", "the Spark model")
            wav = _oneshot_say(text)
        if not wav:
            sys.exit("synthesis failed")
        _play(wav, "preview")
        print(f'  spoke "{text}"  (voice: {active()})')
        return

    # Compare: vary temp/voice per call, which the sidecar can't do — render the
    # whole matrix one-shot (model loads once, stays warm across variants).
    _ensure_venv("mlx_audio", "the Spark model")
    import spark_sidecar
    print(f'  comparing {len(vnames)*len(temps)} option(s) for: "{text}"\n')
    for v in vnames:
        rw, rt = _resolve_voice(v)
        for t in temps:
            label = f"voice={v}" + (f"  temp={t}" if t is not None else "")
            print(f"  ▶ {label}")
            _play(spark_sidecar.synth(text, ref_wav=rw, ref_text=rt, temp=t),
                  f"cmp-{v}-{t}")
    print("\n  done — set the winner with:  XCOPILOT_TTS_SIDECAR_TEMP / use <voice>")


def _resolve_voice(name: str):
    """(ref_wav, ref_text) for a named voice, with legacy fallback."""
    wav = os.path.join(VOICES_DIR, name, "ref.wav")
    txt = os.path.join(VOICES_DIR, name, "ref.txt")
    if not os.path.exists(wav):
        legacy = os.path.join(HERE, "roger_voice_ref.wav")
        if name == "roger" and os.path.exists(legacy):
            return legacy, open(os.path.join(HERE, "roger_voice_ref.txt")).read().strip()
        sys.exit(f"no such voice: {name!r} (see: voices.py list)")
    return wav, open(txt).read().strip()


def _play(wav: bytes, tag: str):
    if not wav:
        print("    (synthesis failed)")
        return
    out = f"/tmp/x-copilot-{tag}.wav"
    with open(out, "wb") as f:
        f.write(wav)
    subprocess.run(["afplay", out])


def _venv_python():
    """Find an mlx-audio venv python (matches the Rust sidecar's discovery)."""
    for c in (os.environ.get("XCOPILOT_TTS_SIDECAR_PYTHON", ""),
              "/tmp/mlxtts/bin/python", ".venv/bin/python", "voice/.venv/bin/python"):
        if not c:
            continue
        p = c if os.path.isabs(c) else os.path.join(ROOT, c)
        if os.path.exists(p):
            return p
    return None


def _ensure_venv(module: str, label: str):
    """If the current interpreter lacks `module`, re-exec this script under a
    discovered mlx-audio venv — so plain `python3 voice/voices.py …` just works
    for commands that need the model (say) or the mic (add --mic)."""
    import importlib.util
    if importlib.util.find_spec(module) is not None:
        return  # already runnable here
    if os.environ.get("_XCOPILOT_VOICES_REEXEC"):
        sys.exit(f"need {label} ({module}), but the venv python doesn't have it. "
                 "Set XCOPILOT_TTS_SIDECAR_PYTHON to a venv that does.")
    py = _venv_python()
    if not py:
        sys.exit(f"need the mlx-audio venv for {label}; none found at "
                 "/tmp/mlxtts/bin/python or .venv. Set XCOPILOT_TTS_SIDECAR_PYTHON.")
    print(f"  (switching to {py})")
    env = dict(os.environ, _XCOPILOT_VOICES_REEXEC="1")
    os.execve(py, [py, os.path.abspath(__file__), *sys.argv[1:]], env)


def _sidecar_say(text: str):
    """POST to a running sidecar; None if it isn't up."""
    import urllib.request, urllib.error
    url = os.environ.get("XCOPILOT_TTS_SIDECAR_URL", "http://127.0.0.1:8765/synth")
    try:
        req = urllib.request.Request(url, data=text.encode("utf-8"), method="POST")
        with urllib.request.urlopen(req, timeout=60) as r:
            print("  (via running sidecar)")
            return r.read()
    except (urllib.error.URLError, OSError):
        return None


def _oneshot_say(text: str):
    """Load the Spark model once and synthesize. Needs the mlx-audio venv."""
    print("  (no sidecar — loading the model one-shot; ~10s)")
    import spark_sidecar  # importing loads the model (top-level); venv ensured above
    return spark_sidecar.synth(text)


def cmd_use(args):
    name = args.name
    if name not in voices():
        sys.exit(f"no such voice: {name!r} (see: voices.py list)")
    os.makedirs(VOICES_DIR, exist_ok=True)
    with open(ACTIVE_FILE, "w") as f:
        f.write(name + "\n")
    print(f"active voice → {name}  (live; next readback uses it)")


def cmd_remove(args):
    name = args.name
    d = os.path.join(VOICES_DIR, name)
    if not os.path.isdir(d):
        sys.exit(f"no such voice: {name!r}")
    shutil.rmtree(d)
    if active() == name and os.path.exists(ACTIVE_FILE):
        os.remove(ACTIVE_FILE)
        print(f"removed {name} (was active → reverted to default)")
    else:
        print(f"removed {name}")


def cmd_add(args):
    name = args.name
    d = os.path.join(VOICES_DIR, name)
    if os.path.isdir(d) and not args.force:
        sys.exit(f"voice {name!r} exists — use `update {name}` (or --force) to overwrite")
    if not args.mic and not args.file:
        sys.exit("choose a source: --mic  or  --file PATH")
    if args.mic:
        _ensure_venv("sounddevice", "microphone recording")  # re-execs under the venv if needed
    os.makedirs(d, exist_ok=True)
    wav_out = os.path.join(d, "ref.wav")
    txt_out = os.path.join(d, "ref.txt")

    # Reference TEXT must accurately match the AUDIO — a mismatch leaks the
    # reference into synthesis. For --mic the script you read IS the ground truth,
    # so use it verbatim (whisper would only inject mis-hearings). For --file the
    # content is unknown, so use --text if given, else auto-transcribe.
    if args.mic:
        # Don't hard-cap a scripted read — the text must cover the whole clip.
        _record_mic(wav_out, args.seconds)
        text = args.text or (open(args.text_file).read() if args.text_file else MIC_SCRIPT)
    else:
        _import_file(args.file, wav_out, args.max_seconds)
        text = _ref_text(wav_out, args, default=None)
    with open(txt_out, "w") as f:
        f.write(text.strip() + "\n")
    print(f"\n  saved voice '{name}' ({_wav_seconds(wav_out):.1f}s)")
    print(f"  switch to it with:  python3 voice/voices.py use {name}\n")


def _record_mic(wav_out: str, seconds: int):
    import numpy as np
    import sounddevice as sd  # available: cmd_add ensured the venv before calling
    print("\n  Read this ALOUD with energy — brisk and upbeat, the way you want")
    print("  the voice to sound. Keep going until recording stops.\n")
    print("  ┌─ SCRIPT ─────────────────────────────────────────────────")
    for line in MIC_SCRIPT.split(". "):
        if line.strip():
            print("  │ " + line.strip().rstrip(".") + ".")
    print("  └──────────────────────────────────────────────────────────\n")
    print(f"  Recording {seconds}s from the default mic.")
    for n in (3, 2, 1):
        print(f"  starting in {n}…", flush=True)
        time.sleep(1)
    print("  ● REC — go!\n", flush=True)
    frames = sd.rec(int(seconds * SR), samplerate=SR, channels=1, dtype="int16")
    sd.wait()
    print("   done.")
    # No hard cap: keep the whole scripted read so the audio matches the script
    # text exactly. Silence head/tail is still trimmed.
    _write_trimmed(wav_out, np.asarray(frames).reshape(-1).tobytes(), SR, 0)


def _import_file(path: str, wav_out: str, max_seconds: int):
    if not os.path.exists(path):
        sys.exit(f"no such file: {path}")
    # Decode/resample anything (mp3/m4a/wav/…) to mono 16-bit 44.1k via afconvert.
    if shutil.which("afconvert") is None:
        sys.exit("need macOS `afconvert` to decode the audio file")
    print(f"  importing {os.path.basename(path)} …")
    tmp = wav_out + ".decoded.wav"
    rc = subprocess.run(
        ["afconvert", path, tmp, "-d", "LEI16@44100", "-c", "1", "-f", "WAVE"]
    ).returncode
    if rc != 0:
        sys.exit("afconvert failed to decode the file")
    with wave.open(tmp) as r:
        frames = r.readframes(r.getnframes())
        sr = r.getframerate()
    os.remove(tmp)
    _write_trimmed(wav_out, frames, sr, max_seconds)


def _write_trimmed(wav_out: str, frames: bytes, sr: int, max_seconds: int, sil: int = 400):
    """Drop near-silent head/tail and cap to max_seconds, snapping the cut to a
    PAUSE so the reference never ends mid-word — a dangling phrase makes Spark
    "continue" it, leaking reference content into synthesis. Writes a mono 16-bit
    WAV. A 0 / negative cap disables the length limit (but the snap still applies
    at the natural end). `sil` is the int16 amplitude below which a window counts
    as a pause."""
    from array import array
    a = array("h")
    a.frombytes(frames)
    n = len(a)
    if n:
        win = max(1, int(0.02 * sr))  # ~20ms analysis windows
        peaks = [max(max(a[w:w + win]), -min(a[w:w + win])) for w in range(0, n, win) if a[w:w + win]]
        loud = [k for k, p in enumerate(peaks) if p > sil]
        if loud:
            padw = max(1, int(0.1 * sr / win))
            sw = max(0, loud[0] - padw)
            ew = min(len(peaks), loud[-1] + padw + 1)
            if max_seconds and max_seconds > 0:
                capw = sw + max(1, int(max_seconds * sr / win))
                if ew > capw:
                    # Snap back from the cap to the nearest pause (within ~3s), so
                    # the clip ends on a silence/sentence boundary, not mid-word.
                    lo = max(sw + 1, capw - int(3 * sr / win))
                    snap = next((w for w in range(min(capw, len(peaks) - 1), lo, -1)
                                 if peaks[w] <= sil), None)
                    ew = snap if snap is not None else capw
                    print(f"  trimmed to {ew * win / sr:.1f}s (was {n / sr:.1f}s)")
            a = a[sw * win: min(n, ew * win)]
    with wave.open(wav_out, "w") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(sr)
        w.writeframes(a.tobytes())


def _ref_text(wav: str, args, default=None) -> str:
    if args.text:
        return args.text
    if args.text_file:
        return open(args.text_file).read()
    text = _auto_transcribe(wav)
    if text:
        print(f"  transcribed: {text!r}")
        return text
    if default is not None:
        print("  (no transcriber available — using the prompt script as reference text)")
        return default
    sys.exit("couldn't auto-transcribe; pass --text \"...\" (what's said in the clip)")


def _auto_transcribe(wav: str) -> str:
    """Transcribe via the built x-copilot binary, if present."""
    for cand in ("target/release/x-copilot", "target/debug/x-copilot"):
        binp = os.path.join(ROOT, cand)
        if os.path.exists(binp):
            try:
                out = subprocess.run([binp, "transcribe", wav], cwd=ROOT,
                                     capture_output=True, text=True, timeout=120)
                if out.returncode == 0:
                    return out.stdout.strip()
            except Exception:
                pass
    print("  (x-copilot binary not built — run `cargo build` for auto-transcribe)")
    return ""


def _wav_seconds(path: str) -> float:
    try:
        with wave.open(path) as w:
            return w.getnframes() / w.getframerate()
    except Exception:
        return 0.0


def main():
    p = argparse.ArgumentParser(description="Manage x-copilot cloned voices.")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list").set_defaults(func=cmd_list)
    u = sub.add_parser("use"); u.add_argument("name"); u.set_defaults(func=cmd_use)
    s = sub.add_parser("say")
    s.add_argument("text", nargs="?")
    s.add_argument("--temps", help="compare temperatures, e.g. 0.5,0.6,0.7")
    s.add_argument("--voices", help="compare voices, e.g. roger,Duane (default: active)")
    s.set_defaults(func=cmd_say)
    r = sub.add_parser("remove"); r.add_argument("name"); r.set_defaults(func=cmd_remove)
    for verb, force in (("add", False), ("update", True)):
        a = sub.add_parser(verb)
        a.add_argument("name")
        a.add_argument("--mic", action="store_true", help="record from the microphone")
        a.add_argument("--seconds", type=int, default=20, help="mic recording length (default 20)")
        a.add_argument("--file", help="import from an audio file (mp3/m4a/wav/…)")
        a.add_argument("--text", help="reference text for --file (what's said)")
        a.add_argument("--text-file", dest="text_file", help="reference text from a file")
        a.add_argument("--max-seconds", dest="max_seconds", type=int, default=20,
                       help="cap the reference to N seconds of speech (0 = no cap; default 20)")
        a.set_defaults(func=cmd_add, force=force)
    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
