#!/usr/bin/env bash
# A plan of sessions, one after the other, until the robots' batteries run out (Nick, 2026-10-24: "start cautiously
# and increase what we're testing"; nobody is there, so a robot that falls stays off; that is acceptable when the
# stage was testing something important). Started by run-e02-pi.sh --plan, in its tmux window "run".
# Each stage is its own session (its own logs folder) and is uploaded as soon as it ends (logs only, quick); the
# whole recording goes up at the end. Every stage answers its own questions (--auto: the defaults; nobody waits).
#   1. Pi check, 15 min: the robots on their own sensors only (U, C); the session-1 calibration file. Tests the Pi:
#      pictures kept up with, the robots answering the Pi, recording, upload.
#   2. The new features, 45 min: all four conditions (U, C, K, KC): KC's one weighted fix per stop, K surviving
#      the runner's re-find, the camera-drift watch, the quality map, the find heading check.
#   3. Explore on the real board, 30 min: the robots calibrate the camera themselves (the riskiest stage: two robots
#      went over an open edge on 2026-10-23; the room check and crawling rays are new), then runs with it.
#   4. Until the batteries run out: a full accuracy session (all conditions, the session-1 calibration file).
set -u
cd "$(dirname "$0")/../.."
OUT="$1"; shift
ROBOTS="--no-side --robot 192.168.1.250 --robot 192.168.1.252 --robot 192.168.1.253"
CAL="--classic-setup --calib tools/experiment/data/overhead_floorcal_roam1.json"
DAY=$(date +%Y%m%d-%H%M)
stage() {
  local n="$1"; shift
  echo; echo "=== Plan stage $n: $* ==="; date
  python3 tools/experiment/e02.py --on-pi --auto $ROBOTS "$@"
  echo "=== Stage $n ended (exit $?) ==="; date
  python3 tools/pi/upload_session.py --recordings "$OUT" --logs-only || echo "(upload of stage $n's logs failed: it is retried at the end)"
  sleep 20
}
stage 1 $CAL --own-sensor-only --hours 0.25
stage 2 $CAL --hours 0.75
stage 3 --hours 0.5
stage 4 $CAL --hours 4
echo; echo "=== Plan done: uploading the recording ==="
for d in $(ls -td logs/e02-* 2>/dev/null | grep -v dryrun | head -4); do
  python3 tools/pi/upload_session.py --recordings "$OUT" --logs-only --session "$d" || true
done
python3 tools/pi/upload_session.py --recordings "$OUT" --stop-recorder --no-logs --tag "e02-pi-plan-$DAY-recording"
