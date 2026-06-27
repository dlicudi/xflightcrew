#!/usr/bin/env python3
"""Record a mono 16 kHz clip from the default mic — for the speaker-ID spike and,
later, voice enrollment.

    pip install sounddevice          (soundfile already comes with resemblyzer)
    python3 bench/record_clip.py <out.wav> [seconds]

First run on macOS may prompt for microphone permission for the terminal; allow
it and re-run. Speak a few cockpit commands so the clip is representative.
"""

import sys


def main(argv: list[str]) -> int:
    if not argv:
        print("usage: python3 bench/record_clip.py <out.wav> [seconds]")
        return 2
    out = argv[0]
    seconds = float(argv[1]) if len(argv) > 1 else 5.0

    try:
        import sounddevice as sd
        import soundfile as sf
    except ImportError as e:
        print(f"missing dependency ({e}). Install with: pip install sounddevice", file=sys.stderr)
        return 1

    fs = 16000
    print(f"Recording {seconds:.0f}s to {out} — speak now...")
    audio = sd.rec(int(seconds * fs), samplerate=fs, channels=1)
    sd.wait()
    sf.write(out, audio, fs)
    print(f"saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
