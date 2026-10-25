#!/bin/bash
# rng.sh START LEN OUT : fetch bytes [START, START+LEN) of the joined video from the right part(s)
U=https://github.com/nfmaxfield-cmd/alvik-fleet-capstone/releases/download/e02-session-20261025-154522
P=1992294400
S=$1; L=$2; O=$3; E=$((S+L-1))
: > $O
if [ $S -lt $P ]; then e=$(( E < P-1 ? E : P-1 )); curl -sSL -r $S-$e $U/cam0_video.avi.part-aa >> $O; fi
if [ $E -ge $P ]; then s=$(( S > P ? S-P : 0 )); curl -sSL -r $s-$((E-P)) $U/cam0_video.avi.part-ab >> $O; fi
