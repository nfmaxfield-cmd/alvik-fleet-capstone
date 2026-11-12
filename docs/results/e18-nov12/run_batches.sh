#!/usr/bin/env bash
# e18 simulator batches (one core, one after another): the blind excursions, the identity drive, the e17 comparison re-run
F="$1"; D="$(cd "$(dirname "$0")" && pwd)"
python3 "$D/blind/batch.py" "$F" "$D/blind/excursions.jsonl" 6 > "$D/blind/log.txt" 2>&1
python3 "$D/outback/batch.py" "$F" "$D/outback/trials.jsonl" 30 > "$D/outback/log.txt" 2>&1
for s in 1 2 3 4 5 6; do for d in 0.45 1.0; do
  python3 "$D/stress/run1.py" $s 0.5 $d "$D/stress/e18-$s-$d.json" "$F" >> "$D/stress/log.txt" 2>&1
done; done
rm -rf "$D/outback/__pycache__" "$D/blind/__pycache__"
echo DONE > "$D/batches.done"
