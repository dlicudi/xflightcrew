# Instructor: handbook questions (#87), design note, 2026-09-28

Not built. This note chooses how the instructor answers a student's
handbook question ("why do we use right rudder on the takeoff?", "what is
density altitude?", "what does Vy mean?") before anything is written.

## The rule it lives under

CLAUDE.md: "No LLM, local or cloud ... A model returns only as the
instructor answering handbook questions, never a fallback." So a model is
ALLOWED here and nowhere else, and only in `mode::Instructor`. Allowed is
not required: the cheapest thing that answers the questions students
actually ask wins.

## What is on disk

- `indexes/general_aviation.json` (untracked, 62 MB): 6,504 chunks from
  seven FAA handbooks (PHAK 1,871, Instrument Flying 1,169, Aviation
  Weather 1,081, Airplane Flying 998, Instrument Procedures 857, Weight and
  Balance 279, Risk Management 249). Each chunk is `{source, text,
  embedding}`: text median 818 characters (max 6,372; 6.0 MB of text in
  all), a 768-dimension `nomic-embed-text` embedding.
- `manuals/faa/*.pdf` (untracked, 482 MB): the seven source PDFs. The AIM
  was never ingested.
- Per-aircraft POM indexes (`cirrus_sr22`, `king_air_c90b`,
  `lancair_evolution`), same format.
- The code that built and read them (`rag.rs`, `llm.rs`, `xvl index/ask`)
  was removed with the LLM on 2026-09-13 (dev-history "The LLM retired, and
  a night of rules"). The embeddings came from Ollama: a QUERY must be
  embedded by the same model to use them, so the stored vectors are only
  usable with an embedding model at runtime.

## What the frame rate costs (measured, 2026-07-08)

Sampled `sim/operation/misc/frame_rate_period` over the Web API with each
load in isolation (memory "fps drops = GPU tenants, not TTS"):

- **Ollama generation on Metal (qwen3:8b)**: median 60 fps fell to 30, with
  sustained 20 fps pins for the whole generation. First answer of a flight
  about 10 s (cold prefill of a 3k-token prompt), warm answers 1.5 to 2.7 s.
  CPU-only (`GPU_LAYERS=0`) spares the frames but the cold prefill is slow.
- **ONNX on the CPU (TTS synthesis)**: no measurable fps cost at 4 or 18
  threads.
- **whisper on Metal**: the fps dips of 2026-07-08 were its GPU decode;
  moving it to the CPU (`XCOPILOT_WHISPER_GPU=false`) removed them at 25 %
  more latency.

The lesson: anything on the GPU is taken from X-Plane's frames; CPU work on
the 18-core machine is close to free.

## Options

### A. Deterministic Q&A (closed vocabulary, authored answers)

A table of teaching questions keyed by topic words, each answer a short
authored line that cites its handbook chapter and fills the airframe's own
numbers from the profile (the path `conversation::Response::Reference` and
`reference_limits` already take for "what's Vy"). Seed it from the lessons
the instructor already teaches: every `Task` gets its "why" and "how" (the
takeoff's rudder, the climb's Vy against Vx, the steep turn's overbank and
back pressure, slow flight's left-turning tendencies, the pattern's
spacing, the flare), plus the ACS knowledge items the brief names.

- Frame cost: none. Latency: microseconds. Deterministic and testable as a
  plain `cargo test`, transcripts as tests, exactly the project's ratchet
  (`xvl analyze` lists the missed phrasings, each becomes an entry).
- Cost: authoring. Coverage is what has been written; an unknown question
  gets an honest "that's one for the handbook: PHAK chapter N" pointer, not
  a guess.

### B. Lexical retrieval over the handbook text, answered EXTRACTIVELY

BM25 (or a plain TF-IDF) over the 6,504 chunk TEXTS already on disk (the
embeddings ignored), picking the best one or two SENTENCES of the best
chunk and reading them aloud with the source named ("The Airplane Flying
Handbook says: ...").

- Frame cost: none (CPU, a few ms over 6 MB; the index loads in about a
  second, off the hot path, in instructor mode only).
- Deterministic: the same words give the same passage, so it is testable
  with fixed questions against fixed chunks.
- Cost and risk: PDF-extracted text (running heads, figure captions,
  hyphenation) must be cleaned; a median chunk is about 50 s of speech, so
  sentence selection is the real work; a lexical hit can be the wrong
  sense of a word ("stall" the aerodynamic event vs "stall" in a turn
  discussion). Hit rate is UNMEASURED; it needs a question bench first.
- Needs a small ingest path back (the chunks can be read from the existing
  JSON; nothing needs Ollama).

### C. Semantic retrieval with the existing embeddings, still extractive

Embed the question with `nomic-embed-text` (137 M parameters) and rank the
stored vectors by cosine; answer extractively as in B.

- It IS a model at runtime (the embedder), though not a generator. Run as
  ONNX on the CPU through the `ort` crate the TTS already links, it should
  cost no frames (the TTS measurement), but that is a new measurement to
  make, and via Ollama on Metal it is a GPU tenant again.
- Better recall than B on paraphrases ("why does the nose swing left" to
  the left-turning tendencies section). Same extraction problem as B.

### D. Retrieval plus a local generative model (the old answer path)

C plus a local LLM composing the answer from the retrieved chunks.

- Frame cost: the measured one, median 60 to 30 fps with 20 fps pins, on
  Metal; on the CPU a slow cold prefill (seconds to tens of seconds).
  Warm answers 1.5 to 2.7 s on Metal.
- Non-deterministic, untestable as a plain `cargo test`, able to say
  something the handbook does not; the one thing an instructor must never
  do. It is the path the pilot removed on 2026-09-13 ("measured useless and
  a frame-rate cost").

## Recommendation

**A first, then B behind it; no model.**

1. Build A as the instructor's question table (`instructor::answer`, a
   closed vocabulary like `request`), seeded from the nine tasks' teaching
   points and the brief's numbers, answering only in instructor mode. Every
   question heard is blackboxed (`instructor` `question` with `hit`/`miss`
   and the words), so the misses become entries.
2. Add B as the miss path for handbook-shaped questions only ("what is",
   "why does", "explain"): lexical retrieval over the existing chunk text,
   one or two sentences, the source named, and blackboxed with the chunk id
   so a wrong passage can be pinned by a test. Build a 50-question bench
   (the ACS knowledge areas) and ship B only if its top passage is right on
   a clear majority of it; otherwise A's honest pointer stays the answer.
3. Keep C and D shelved. Reconsider C only if B's bench shows paraphrase
   misses dominate, and then only as ONNX on the CPU with a before/after
   fps measurement (`frame_rate_period`, the 2026-07-08 method) in the
   commit. D stays out: it costs the frames the sim needs and it can invent
   an answer, which is worse than no answer in a teaching persona.

Either way the answer is never spoken in a critical phase (the monitor's
`quiet` gate and the flow's sterile floor apply) and never outside
instructor mode: the copilot's question path stays the deterministic
answerers it is today.

## Owed before building

- A question bench: real student questions, each with the handbook
  chapter that answers it (the test corpus for both A and B).
- A cleaned chunk-text reader for `indexes/general_aviation.json` (text
  only; the embeddings unused under A and B).
- The AIM, if airspace and radio questions matter (never ingested).
