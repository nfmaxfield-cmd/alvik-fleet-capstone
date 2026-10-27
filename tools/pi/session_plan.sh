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
if [ "${1:-}" = "right" ]; then
  # Nick, 2026-10-24 evening: Left and Center went over the edge in stage 3; Right is the one robot left on the
  # board. Only Right is driven (--robots Right; the others, if still switched on, are only asked their battery:
  # driving against idle drain). All four conditions only; explore is its own plan (right-explore, its fix: a leg's move is its done pose
  # less its started pose), run once the virtual border protects it. The fit limit is 2.5 cm for these (stage 2: 2.1-2.5
  # with the session-1 calibration file, all refused at 1.5; chosen, the sensor-trust build replaces it).
  EXTRA="--robots Right --fit-max-rms-cm 2.5 --battery-watch-s 300"
  stage R1 $CAL $EXTRA --hours 1
  # explore waits for the virtual border (Nick, 2026-10-24 18:42): ./run-e02-pi.sh --plan right-explore later
fi
if [ "${1:-}" = "right-explore" ]; then
  EXTRA="--robots Right --fit-max-rms-cm 2.5 --battery-watch-s 300"
  stage R2 $EXTRA --calib tools/experiment/data/overhead_floorcal_roam1.json --hours 0.5   # the fallback if explore fails (still explore: no --classic-setup)
fi
if [ "${1:-}" = "edges" ]; then
  # Nick, 2026-10-26 22:49: both boards taken off for edge data (Monday's school table has none) and footage for
  # the two-camera work (start with --with-side). All four sides are open edges from the start (--no-boards: every
  # find feels for two edges; the stored calibration, no explore, which assumes two boards). Nobody is there: a
  # robot that goes off stays off.
  #   E1, 15 min (chosen; a check, not a measurement): the robots on their own sensors (U, C): do edge finds work
  #       with no boards at all before the cameras' conditions join.
  #   E2, until the batteries run out: all conditions; every run starts with an edge find.
  #   Holds: nobody answers at night, and an unanswered hold waits --answer-timeout-min before the robot rescues
  #   itself (default 30 min, set for when Nick is there). 3 min here (chosen convenience setting, not measured; no
  #   DOE needed: it decides how long the night idles per hold, not anything measured), so 4 open edges' border holds
  #   don't eat the night.
  NB="$CAL --no-boards --answer-timeout-min 3"
  stage E1 $NB --own-sensor-only --hours 0.25
  stage E2 $NB --hours 4 --hard-cap-min 0
fi
if [ -z "${1:-}" ]; then
  stage 1 $CAL --own-sensor-only --hours 0.25
  stage 2 $CAL --hours 0.75
  stage 3 --hours 0.5
  # the final stage runs until the batteries run out: no hard cap (Nick, 2026-10-26); the no-answer timeouts still apply
  stage 4 $CAL --hours 4 --hard-cap-min 0
fi
echo; echo "=== Plan done: uploading the recording ==="
for d in $(ls -td logs/e02-* 2>/dev/null | grep -v dryrun | head -$(case "${1:-}" in "") echo 4;; edges) echo 2;; *) echo 1;; esac)); do
  python3 tools/pi/upload_session.py --recordings "$OUT" --logs-only --session "$d" || true
done
python3 tools/pi/upload_session.py --recordings "$OUT" --stop-recorder --no-logs --tag "e02-pi-plan-$DAY-recording"
