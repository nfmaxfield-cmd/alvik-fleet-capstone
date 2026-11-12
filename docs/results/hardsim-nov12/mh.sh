#!/bin/sh
B=/tmp/claude-0/-home-claude/fff6dc9c-5876-54f9-92a7-a08483d3d5c7/scratchpad/hardsim/tools/fleet/hardsim_batch.py
C=$1; O=$2
python3 $B --profile measured --seeds 1-8 --hours 0.5 --jobs 2 --code $C --out $O/measured
python3 $B --profile harsh --seeds 1-8 --hours 0.5 --jobs 2 --code $C --out $O/harsh
echo ALLDONE
