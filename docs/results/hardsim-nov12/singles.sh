#!/bin/sh
B=/tmp/claude-0/-home-claude/fff6dc9c-5876-54f9-92a7-a08483d3d5c7/scratchpad/hardsim/tools/fleet/hardsim_batch.py
C=$1; O=$2
for f in hung restart wrong_names missing_cam calib5 dark skew latency2 bursts; do
  python3 $B --profile measured --faults $f --seeds 1-2 --hours 0.5 --jobs 2 --code $C --out $O/single
done
echo ALLDONE
