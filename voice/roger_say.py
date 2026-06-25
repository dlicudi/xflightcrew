#!/usr/bin/env python3
"""Speak text in the cloned Roger voice (Spark-TTS, warm).

  one-shot:     python voice/roger_say.py "Roger, flaps up"
  interactive:  python voice/roger_say.py        # then type lines

Renders sentence-by-sentence (Spark drifts on long single generations) at
temperature 0.6, the tuned setting. Reference: voice/roger_voice_ref.{wav,txt}.
Run with the mlx-audio venv, e.g. /tmp/mlxtts/bin/python.
"""
import sys, re, os, glob, subprocess
from mlx_audio.tts.utils import load_model
from mlx_audio.tts.generate import generate_audio

MODEL = "mlx-community/Spark-TTS-0.5B-bf16"
HERE = os.path.dirname(os.path.abspath(__file__))
REF = os.path.join(HERE, "roger_voice_ref.wav")
REF_TEXT = open(os.path.join(HERE, "roger_voice_ref.txt")).read().strip()
OUT = "/tmp/roger_say_out"

def say(text, model):
    for s in re.split(r"(?<=[.!?])\s+", text.strip()):
        s = s.strip()
        if not s:
            continue
        for f in glob.glob(OUT + "*.wav"):
            os.remove(f)
        generate_audio(text=s, model=model, ref_audio=REF, ref_text=REF_TEXT,
                       temperature=0.6, file_prefix=OUT, audio_format="wav",
                       join_audio=True, save=True, verbose=False)
        files = sorted(glob.glob(OUT + "*.wav"))
        if files:
            subprocess.run(["afplay", files[0]])

print("Loading the Roger voice (one-time, ~10s)…", flush=True)
model = load_model(MODEL)
args = " ".join(sys.argv[1:]).strip()
if args:
    say(args, model)
else:
    print('Ready. Type a line, Enter to hear it in your voice. Ctrl-C to quit.')
    while True:
        try:
            line = input("roger> ")
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if line.strip():
            say(line, model)
