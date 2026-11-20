#!/bin/bash
# 20 seeds x 3 configs, interleaved by seed, one at a time; resumes from results.csv
T=/tmp/claude-0/-home-claude/fff6dc9c-5876-54f9-92a7-a08483d3d5c7/scratchpad/turnback20
SR=$T/wt_ccb3/docs/results/e23-school-fixes/seed_runs.py
R=$T/runs; mkdir -p $R
OPTS="late=20,1.5:slow=6:skew=0:pts=4:wall=600"
for s in $(seq 1 20); do
  for c in a b c; do
    name=$c-s$s
    if [ -f $T/results.csv ] && grep -q ",$name\$" $T/results.csv; then continue; fi
    if [ -d $R/$name ] && [ ! -f $R/$name/case.json ]; then rm -rf $R/$name $R/$name-home $R/$name.json; fi
    case $c in
      a) repo=$T/wt_b202; extra="";;
      b) repo=$T/wt_ccb3; extra="";;
      c) repo=$T/wt_ccb3; extra=":env=MIRLT_OLD_FIX_JUMPS=count";;
    esac
    echo "$(date +%T) start $name" >> $T/batch.log
    ( cd $repo && python3 $SR $repo $R $name:$s:$OPTS$extra >> $T/batch.log 2>&1 )
    python3 $T/scripts/row.py $c $s $R/$name >> $T/batch.log 2>&1
  done
done
echo done > $T/batch.done
