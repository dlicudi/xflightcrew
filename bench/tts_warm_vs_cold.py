#!/usr/bin/env python3
"""TTS warm-vs-cold session benchmark.

Usage (from repo root; needs models/en_US-amy-medium.onnx + espeak-ng on PATH):
    python3 -m venv /tmp/ttsbench && /tmp/ttsbench/bin/pip install onnxruntime numpy
    /tmp/ttsbench/bin/python bench/tts_warm_vs_cold.py

Measure the TTS architecture variable that actually matters for x-copilot:
warm in-process session (load once, reuse) vs cold reload-per-utterance
(what a subprocess piper backend pays every call).

Uses the piper VITS model already vendored in models/. Rust's `ort` wraps the
SAME libonnxruntime, so these inference timings are a faithful proxy for an
in-process Rust backend. We measure the *delta*; absolute numbers track the
machine.
"""
import json, subprocess, time, statistics, resource, sys
import numpy as np
import onnxruntime as ort

MODEL = "models/en_US-amy-medium.onnx"
CFG = json.load(open(MODEL + ".json"))
SR = CFG["audio"]["sample_rate"]
PMAP = CFG["phoneme_id_map"]
INF = CFG["inference"]
PHRASES = [
    "Flaps up.",
    "Roger, heading two seven zero, autopilot engaged.",
    "Gear is down, three green.",
    "Caution, fuel endurance forty minutes.",
]

def phonemize(text):
    # espeak-ng IPA, the same phonemizer piper uses. Good enough for representative
    # token-sequence length (what drives inference time).
    out = subprocess.run(
        ["espeak-ng", "-v", "en-us", "-q", "--ipa", text],
        capture_output=True, text=True).stdout
    return out.replace("\n", " ").strip()

def to_ids(phonemes):
    ids = list(PMAP["^"])              # BOS
    for ch in phonemes:
        if ch in PMAP:
            ids += PMAP[ch]
            ids += PMAP["_"]           # pad between phonemes
    ids += list(PMAP["$"])             # EOS
    return ids

def make_inputs(text):
    ids = to_ids(phonemize(text))
    return {
        "input": np.array([ids], dtype=np.int64),
        "input_lengths": np.array([len(ids)], dtype=np.int64),
        "scales": np.array([INF["noise_scale"], INF["length_scale"], INF["noise_w"]],
                           dtype=np.float32),
    }, len(ids)

def synth(sess, inp):
    audio = sess.run(None, inp)[0].squeeze()
    return audio.shape[-1] / SR  # seconds of speech produced

def new_session(provider):
    so = ort.SessionOptions()
    so.log_severity_level = 4
    return ort.InferenceSession(MODEL, sess_options=so, providers=[provider])

def bench(provider):
    print(f"\n=== provider: {provider} ===")
    inputs = [make_inputs(p) for p in PHRASES]

    t0 = time.perf_counter()
    sess = new_session(provider)
    load_ms = (time.perf_counter() - t0) * 1000
    synth(sess, inputs[0][0])  # warm-up (first run JITs kernels)
    print(f"one-time session load:      {load_ms:7.1f} ms")

    # WARM: reuse the session (the proposed in-process design)
    warm = []
    for inp, n in inputs:
        t = time.perf_counter(); dur = synth(sess, inp)
        ms = (time.perf_counter() - t) * 1000
        warm.append((ms, dur, n))

    # COLD: a fresh session per utterance (isolates the reload tax a subprocess pays)
    cold = []
    for inp, n in inputs:
        t = time.perf_counter()
        s = new_session(provider); synth(s, inp)
        cold.append((time.perf_counter() - t) * 1000)

    print(f"{'phrase':<52}{'toks':>5}{'speech':>8}{'WARM':>9}{'COLD':>9}{'RTF':>7}")
    for (wms, dur, n), cms, p in zip(warm, cold, PHRASES):
        print(f"  {p:<50}{n:>5}{dur:>7.2f}s{wms:>8.1f}ms{cms:>8.1f}ms{wms/1000/dur:>7.3f}")
    wmean = statistics.mean(w[0] for w in warm)
    cmean = statistics.mean(cold)
    print(f"  {'mean':<50}{'':>5}{'':>8}{wmean:>8.1f}ms{cmean:>8.1f}ms")
    print(f"  → cold pays {cmean/wmean:.1f}x the warm cost per utterance "
          f"(+{cmean-wmean:.0f} ms reload tax)")
    rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e6
    print(f"  resident memory (warm session held): ~{rss:.0f} MB")

if __name__ == "__main__":
    # One provider per process: the CoreML EP can segfault at teardown on this
    # ORT build, so isolate it — a crash in one can't lose the other's results.
    if len(sys.argv) > 1:
        bench(sys.argv[1])
    else:
        import os
        for prov in ["CPUExecutionProvider", "CoreMLExecutionProvider"]:
            rc = subprocess.call([sys.executable, "-u", __file__, prov],
                                 stderr=subprocess.DEVNULL)
            if rc != 0:
                print(f"\n=== provider: {prov} ===\n  child exited {rc} "
                      f"(CoreML EP teardown crash — known ORT issue; CPU result above stands)")
