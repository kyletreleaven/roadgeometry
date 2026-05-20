#!/bin/bash
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
BENCH="${DIR}/../setiptah-roadgeometry-matching-cpp/build/bench_matching"
INSTANCE="${DIR}/captured_instance.json"
STACKS="${DIR}/bench_matching.stacks"
OUT="${DIR}/bench_matching.svg"
REPEATS=${REPEATS:-5}

if [ ! -f "$INSTANCE" ]; then
    echo "Dumping captured instance..."
    nox -s bench -- bench/dump_captured.py
fi

sudo -v
"$BENCH" "$INSTANCE" "$REPEATS" &
PID=$!
sudo sample $PID 60 -f "$STACKS"
wait $PID

/opt/homebrew/bin/stackcollapse-sample.awk "$STACKS" \
  | /opt/homebrew/bin/flamegraph.pl --title "bench_matching" --width 2400 > "$OUT"

# open "$OUT"
