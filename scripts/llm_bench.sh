#!/usr/bin/env zsh
#
# Battery-test the local LLM tool-picker across one or more Ollama models.
#
# Runs an ignored integration test (which needs a live `ollama serve`) once per
# model, tallying the per-case ✓/✗ marks it prints into a pass/fail/rate/time
# table. Use it to compare models before picking a default (see src/llm.rs).
#
# Usage:
#   scripts/llm_bench.sh qwen3:8b qwen2.5:7b-instruct llama3.1:8b
#   TEST=llm_picks_tool_noisy scripts/llm_bench.sh qwen3:8b      # adversarial set
#
# TEST selects the battery (default: llm_picks_correct_tool, the clean set;
# llm_picks_tool_noisy is the real-world/adversarial set). Results and per-model
# logs land in target/llm_bench/ (gitignored).
set -u
zmodload zsh/datetime

# Resolve the repo root from this script's location (scripts/ sits at the root),
# so the bench works regardless of the caller's working directory.
REPO=${0:A:h:h}
cd "$REPO" || { echo "cannot cd to repo root: $REPO" >&2; exit 1 }

if (( $# == 0 )); then
  echo "usage: $0 <model> [model ...]   (e.g. qwen3:8b qwen2.5:7b-instruct)" >&2
  echo "       TEST=llm_picks_tool_noisy $0 <model>   # adversarial battery" >&2
  exit 1
fi

OUT="$REPO/target/llm_bench"
mkdir -p "$OUT"
TEST="${TEST:-llm_picks_correct_tool}"
SUMMARY="$OUT/summary_${TEST}.txt"
: > "$SUMMARY"
echo "battery: $TEST"

printf "%-26s %6s %6s %8s %9s\n" "MODEL" "PASS" "FAIL" "RATE" "TIME(s)" | tee -a "$SUMMARY"
printf '%.0s-' {1..62} | tee -a "$SUMMARY"; echo | tee -a "$SUMMARY"

for m in "$@"; do
  log="$OUT/${TEST}__${m//[:\/]/_}.log"
  # qwen3:Nb are hybrid reasoning models; disable thinking for a fair latency
  # read. Match `qwen3:` only — qwen3-coder is non-reasoning and rejects `think`.
  nothink=""
  [[ "$m" == qwen3:* ]] && nothink=1
  start=$EPOCHSECONDS
  XCOPILOT_MODEL="$m" XCOPILOT_NO_THINK="$nothink" \
    cargo test --bin x-copilot "$TEST" -- --ignored --nocapture \
    > "$log" 2>&1
  elapsed=$(( EPOCHSECONDS - start ))
  pass=$(grep -c '✓' "$log")
  fail=$(grep -c '✗' "$log")
  total=$(( pass + fail ))
  if (( total > 0 )); then rate=$(( 100 * pass / total )); else rate=0; fi
  printf "%-26s %6d %6d %7d%% %9d\n" "$m" "$pass" "$fail" "$rate" "$elapsed" | tee -a "$SUMMARY"
done

echo | tee -a "$SUMMARY"
echo "Per-model logs in $OUT/" | tee -a "$SUMMARY"
