#!/usr/bin/env bash
# The whole E02 session on the Raspberry Pi (Nick, 2026-10-24): start it, close the laptop, leave the house.
# The laptop way (run-e02.bat) still works exactly as before; this is the other choice.
#
# In a terminal ON THE PI (RealVNC, or SSH from the laptop):
#     cd ~/alvik-fleet-capstone && ./run-e02-pi.sh
# Extra runner options go on the end, e.g.  ./run-e02-pi.sh --hours 1 --marks
#
# What it does:
#   1. checks the tools (tmux, OpenCV, SciPy, Matplotlib) and where the video goes: the USB drive if one is plugged in
#      (exFAT or ext4; FAT32 can't hold the video), else asks before using the Pi's own card;
#   2. starts the camera recorder and then the runner, each in its own window of a tmux session called e02,
#      so they keep running when this terminal is closed;
#   3. when the runner finishes: stops the recorder, and uploads the logs and the video to a GitHub release
#      (tools/pi/upload_session.py) so they can be fetched from anywhere.
# Watch and answer questions from any phone or computer on the home WiFi: http://st1maxfini.local:8090
# Back to the windows later:  tmux attach -t e02      Leave them running: Ctrl+B, then D.
#
# A plan of sessions until the batteries run out, nobody there (tools/pi/session_plan.sh):
#     ./run-e02-pi.sh --plan
# A production session (2026-10-25: no video; the tracker opens the camera itself and keeps a clip and a still of
# every hold, border or edge stop, help call, Note and failed turn, plus a thumbnail every 30 s, in the session's
# frames/ folder):
#     ./run-e02-pi.sh --production
# Cores (2026-10-25, a 4-core Pi): the runner and the live page on core 0, the recorder on core 1, the camera tracker
# (its own process, started by the runner) on cores 2-3. cpu.csv in the session folder shows each core's load.
set -u
cd "$(dirname "$0")"
PLAN=0
# --with-side: record the side camera too (Nick, 2026-10-24: footage for its calibration and the two-camera
# tracker; the runner does not use it yet). Found by its name (tools/pi/camnames.py); SIDE_DEV=<number> overrides. Recording only: the overhead's picture rate is in perf.csv to check.
SIDE=0
PROD=0
for a in "$@"; do [ "$a" = "--with-side" ] && SIDE=1; [ "$a" = "--production" ] && PROD=1; done
if [ "$SIDE" = 1 ]; then
  set -- $(for a in "$@"; do [ "$a" != "--with-side" ] && printf '%s ' "$a"; done)
fi
PLAN_NAME=""
if [ "${1:-}" = "--plan" ]; then
  PLAN=1; shift
  # an optional plan name: ./run-e02-pi.sh --plan right (Right alone, tools/pi/session_plan.sh)
  case "${1:-}" in right|right-explore|edges) PLAN_NAME="$1"; shift;; esac
fi
REPO="$(pwd)"
RUN_ARGS_DEFAULT="--no-side --robot 192.168.1.250 --robot 192.168.1.252 --robot 192.168.1.253"
# Cameras by name (Nick, 2026-10-26: after a Pi restart the numbers swapped and the runner opened the side camera as
# the overhead). tools/pi/camnames.py finds each camera's number from its name (~/.config/mirlt/cameras.json says
# which name is the overhead and which the side); `python3 tools/pi/camnames.py` shows what it sees.
# OVERHEAD_DEV / SIDE_DEV in front still pick a number by hand, e.g.  OVERHEAD_DEV=2 ./run-e02-pi.sh
cam_dev() {   # cam_dev overhead|side: the number, or a plain message and stop
  local d
  local var=OVERHEAD_DEV; [ "$1" = side ] && var=SIDE_DEV
  d="$(python3 tools/pi/camnames.py --dev "$1")" || {
    echo "Stopped: the $1 camera was not found by its name (the line above says what the Pi sees). Plug it in and" >&2
    echo "start again, or give its number by hand, e.g.  $var=2 ./run-e02-pi.sh" >&2
    exit 1
  }
  echo "$d"
}
if [ -n "${OVERHEAD_DEV:-}" ]; then OD="$OVERHEAD_DEV"; echo "Overhead camera: /dev/video$OD (OVERHEAD_DEV, by hand)."
else OD="$(cam_dev overhead)" || exit 1; echo "Overhead camera: /dev/video$OD (found by its name)."; fi
CAM_ARGS="--device $OD --names overhead --fps 25 --label E02 --no-lock"
if [ "$SIDE" = 1 ]; then
  if [ -n "${SIDE_DEV:-}" ]; then SD="$SIDE_DEV"
  else SD="$(cam_dev side)" || exit 1; fi
  if [ ! -e "/dev/video$SD" ]; then
    echo "No side camera at /dev/video$SD. Plug it in, or find its number with:  python3 tools/pi/camnames.py"
    echo "then start again with SIDE_DEV=<number> in front, e.g.  SIDE_DEV=4 ./run-e02-pi.sh --with-side"
    exit 1
  fi
  if [ "$SD" = "$OD" ]; then echo "The overhead and side cameras are both /dev/video$SD. Stopped."; exit 1; fi
  CAM_ARGS="--device $OD,$SD --names overhead,side --fps 25 --label E02 --no-lock"
  echo "Recording the side camera too (/dev/video$SD); the runner uses the overhead only."
fi

NCPU=$(nproc 2>/dev/null || echo 1)
RUN_PIN=""; REC_CORES=""
if [ "$NCPU" -ge 4 ] && command -v taskset >/dev/null; then
  RUN_PIN="taskset -c 0"; REC_CORES="--cores 1"
  [ "$SIDE" = 1 ] && REC_CORES="--cores 0,1"     # two cameras: the recorder shares core 0 with the runner
  [ "$PROD" = 1 ] && RUN_PIN="taskset -c 0,1"    # no recorder: the runner gets its core too
fi
if [ "$PROD" = 1 ] && [ "$SIDE" = 1 ]; then
  echo "--production records no video, so --with-side does nothing there. Stopped."; exit 1
fi

echo "=== E02 on the Pi ==="
need=""
command -v tmux >/dev/null || need="$need tmux"
python3 -c "import cv2, numpy" 2>/dev/null || need="$need python3-opencv python3-numpy"
python3 -c "import scipy" 2>/dev/null || need="$need python3-scipy"
python3 -c "import matplotlib" 2>/dev/null || need="$need python3-matplotlib"
if [ -n "$need" ]; then
  echo "Installing:$need (needs the Pi's password once)"
  sudo apt install -y $need || { echo "Could not install$need. Stopped."; exit 1; }
fi

if tmux has-session -t e02 2>/dev/null; then
  echo "A session is already running on this Pi. To see it:  tmux attach -t e02"
  exit 1
fi

# where the video goes (none in a production session: frames go into the session folder)
OUT=""
if [ "$PROD" = 1 ]; then
  free_gb=$(df -BG --output=avail "$REPO" | tail -1 | tr -dc 0-9)
  if [ "${free_gb:-0}" -lt 3 ]; then
    echo "Only $free_gb GB free on the Pi's card; a production session's clips need up to about 3 GB. Stopped."; exit 1
  fi
  echo "Production session: no video; clips and stills go to the session folder (logs/e02-*/frames)."
else
for m in /media/"$USER"/*; do
  [ -d "$m" ] && [ -w "$m" ] || continue
  fs="$(findmnt -no FSTYPE -T "$m" 2>/dev/null)"
  if [ "$fs" = "vfat" ]; then
    echo "The USB drive at $m is FAT32, which can't hold a file over 4 GB (one session's video is about 5 GB)."
    echo "Reformat it as exFAT (docs/pi-run.md, 'USB drive'), or unplug it to record to the Pi's card."
    exit 1
  fi
  OUT="$m/recordings"
  break
done
if [ -z "$OUT" ]; then
  echo "No USB drive found (nothing under /media/$USER)."
  if [ -n "${E02_LAUNCHER:-}" ]; then
    # started from the start page: no terminal to ask in; the page's "record to the Pi's card" choice answers
    ans="${E02_CARD_OK:-n}"
    [ "$ans" = "y" ] || echo "Plug in the USB drive, or tick 'no USB drive: record to the Pi's card' on the start page."
  else
  read -r -p "Record to the Pi's own card instead? Type y and press Enter (anything else stops): " ans
  fi
  [ "$ans" = "y" ] || exit 1
  OUT="$HOME/recordings"
fi
mkdir -p "$OUT" || { echo "Can't write to $OUT. Stopped."; exit 1; }
free_gb=$(df -BG --output=avail "$OUT" | tail -1 | tr -dc 0-9)
echo "Video goes to $OUT ($free_gb GB free)."
# space needed: about 3 GB an hour of video plus 1 GB to spare (session 1: 5.5 GB in 2 h); --hours sets it
hours=2
[ "$PLAN" = 1 ] && hours=5.5
[ -n "$PLAN_NAME" ] && hours=1.5
[ "$PLAN_NAME" = edges ] && hours=4.5
prev=""
for a in "$@"; do [ "$prev" = "--hours" ] && hours="$a"; prev="$a"; done
need_gb=$(python3 -c "import math,sys; print(int(math.ceil(3*float(sys.argv[1])*(1+int(sys.argv[2]))+1)))" "$hours" "$SIDE" 2>/dev/null || echo 13)   # the side camera doubles it
if [ "${free_gb:-0}" -lt "$need_gb" ]; then
  echo "Only $free_gb GB free; a $hours-hour session needs about $need_gb GB. Free some space first. Stopped."
  exit 1
fi
fi
if [ ! -f "$HOME/.config/mirlt/github_token" ]; then
  echo "NOTE: no GitHub token yet (docs/pi-run.md, 'GitHub token'): the session runs, but the upload at the end"
  echo "      is skipped; the files stay on the Pi."
fi
git -C "$REPO" pull --ff-only >/dev/null 2>&1 && echo "Code up to date." || echo "(git pull skipped: offline or local changes)"

if [ "$#" -gt 0 ]; then RUN_ARGS="$RUN_ARGS_DEFAULT $*"; else RUN_ARGS="$RUN_ARGS_DEFAULT"; fi
# a production session's tracker opens the camera itself: give it the overhead's number (before 2026-10-26 it
# always opened /dev/video0, whatever OVERHEAD_DEV said)
[ "$PROD" = 1 ] && RUN_ARGS="$RUN_ARGS --camera-device $OD"
# the start page (tools/pi/launcher.py) keeps port 8090 while no session runs: ask it to let go (nothing happens
# if it is not installed)
python3 tools/pi/launcher.py --handoff >/dev/null 2>&1 || true

if [ "$PROD" = 1 ]; then
  if [ "$PLAN" = 1 ]; then echo "--production and --plan don't go together yet. Stopped."; exit 1; fi
  tmux new-session -d -s e02 -n run "cd '$REPO' && $RUN_PIN python3 tools/experiment/e02.py --on-pi $RUN_ARGS; echo; echo '=== Runner finished: uploading the logs and frames ==='; python3 tools/pi/upload_session.py --recordings '$REPO/logs' --logs-only; echo; read -r -p 'All done. Press Enter to close this window.'"
  tmux select-window -t e02:run
  echo
  echo "Started (production: no video). Watch it on any phone or computer on the WiFi:"
  echo "    http://$(hostname).local:8090"
  [ -n "${E02_LAUNCHER:-}" ] && exit 0
  sleep 2
  tmux attach -t e02
  exit 0
fi
tmux new-session -d -s e02 -n rec "cd '$REPO' && python3 tools/pi/camrec.py $CAM_ARGS $REC_CORES --out '$OUT'; echo; read -r -p 'Recorder stopped. Press Enter to close this window.'"
echo "Recorder starting (8 s) ..."
sleep 8
if [ "$PLAN" = 1 ]; then
  tmux new-window -t e02 -n run "cd '$REPO' && bash tools/pi/session_plan.sh '$OUT' $PLAN_NAME; echo; read -r -p 'Plan done. Press Enter to close this window.'"
else
  tmux new-window -t e02 -n run "cd '$REPO' && $RUN_PIN python3 tools/experiment/e02.py --on-pi $RUN_ARGS; echo; echo '=== Runner finished: stopping the recorder and uploading ==='; python3 tools/pi/upload_session.py --recordings '$OUT' --stop-recorder; echo; read -r -p 'All done. Press Enter to close this window.'"
fi
tmux select-window -t e02:run
echo
echo "Started. Watch it and answer its questions on any phone or computer on the WiFi:"
echo "    http://$(hostname).local:8090"
[ -n "${E02_LAUNCHER:-}" ] && exit 0      # started from the start page: nobody to show the window to
echo "Opening the runner's window now. To leave it running and close this terminal: Ctrl+B, then D."
sleep 2
tmux attach -t e02
