#!/bin/bash
# rng.sh START LEN OUT : fetch bytes [START, START+LEN) of the joined cam0 video from the right part(s) by HTTP range
U=https://github.com/nfmaxfield-cmd/alvik-fleet-capstone/releases/download/e02-session-20261026-095541
P=1992294400
S=$1; L=$2; O=$3; E=$((S+L-1))
: > $O
parts=(aa ab ac ad ae af ag ah ai aj ak al am)
for i in "${!parts[@]}"; do
  a=$((i*P)); b=$((a+P-1))
  if [ $E -lt $a ] || [ $S -gt $b ]; then continue; fi
  s=$(( S > a ? S-a : 0 )); e=$(( E < b ? E-a : P-1 ))
  curl -sSL -r $s-$e $U/cam0_video.avi.part-${parts[$i]} >> $O
done
