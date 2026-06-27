#!/usr/bin/env python3
"""Speaker-ID spike — does a voice embedding cleanly separate the PILOT from
X-Plane ATC (and other speakers)?

This proves the *concept* before we build the real feature. It uses Resemblyzer
(a small, well-known GE2E speaker encoder) because it's the lowest-friction way to
get speaker embeddings in a few lines — preprocessing baked in, raw wav in,
embedding out. The separation result is model-agnostic: if embeddings separate you
from ATC here, they will with the production ONNX model (WeSpeaker / 3D-Speaker
CAM++) we'd run via `ort` in Rust. So this only answers "is the idea sound?".

Setup:
    python3 -m venv .venv-spk && source .venv-spk/bin/activate
    pip install resemblyzer        # pulls torch + librosa (~a few hundred MB)

Get clips (any format librosa reads — wav/m4a/mp3; mono-ish, a few seconds each):
    - 2+ clips of YOUR voice saying cockpit commands (e.g. record with QuickTime,
      or reuse a `voice/` reference clip).
    - 1+ clip of X-Plane ATC (record the sim's ATC audio), and/or another person.

Run:
    python3 bench/speaker_id_spike.py pilot1.wav pilot2.wav atc.wav [more.wav ...]

What to look for (cosine similarity, higher = more alike):
    - pilot-vs-pilot pairs:  HIGH  (same voice)            ~> 0.75 typical
    - pilot-vs-ATC  pairs:   LOW   (you vs a synth voice)  ~< 0.5, likely much lower
    A clear, stable gap between those two groups = the gate will work. Pick an
    accept threshold in the gap (e.g. midway) for the real implementation.
"""

import itertools
import sys


def main(paths: list[str]) -> int:
    if len(paths) < 2:
        print(__doc__)
        print("error: give at least two audio files to compare.", file=sys.stderr)
        return 2

    try:
        import numpy as np
        from resemblyzer import VoiceEncoder, preprocess_wav
    except ImportError as e:
        print(f"missing dependency ({e}). Install with: pip install resemblyzer", file=sys.stderr)
        return 1

    encoder = VoiceEncoder()
    embeddings: dict[str, "np.ndarray"] = {}
    for path in paths:
        try:
            embeddings[path] = encoder.embed_utterance(preprocess_wav(path))
        except Exception as e:  # noqa: BLE001 - spike: report and skip a bad file
            print(f"  ! could not embed {path}: {e}", file=sys.stderr)

    names = list(embeddings)
    if len(names) < 2:
        print("error: need at least two embeddable files.", file=sys.stderr)
        return 1

    # Cosine similarity (Resemblyzer embeddings are L2-normalized, so dot == cosine).
    print("\n  pairwise speaker similarity (1.0 = identical voice):\n")
    width = max(len(short(n)) for n in names)
    for a, b in itertools.combinations(names, 2):
        sim = float(np.dot(embeddings[a], embeddings[b]))
        bar = "█" * round(sim * 20)
        print(f"  {short(a):<{width}}  vs  {short(b):<{width}}   {sim:+.3f}  {bar}")
    print("\n  → want pilot-vs-pilot HIGH and pilot-vs-ATC LOW, with a clear gap.\n")
    return 0


def short(path: str) -> str:
    return path.rsplit("/", 1)[-1]


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
