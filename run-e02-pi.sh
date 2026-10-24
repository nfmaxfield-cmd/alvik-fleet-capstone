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
set -u
cd "$(dirname "$0")"
PLAN=0
if [ "${1:-}" = "--plan" ]; then PLAN=1; shift; fi
REPO="$(pwd)"
RUN_ARGS_DEFAULT="--no-side --robot 192.168.1.250 --robot 192.168.1.252 --robot 192.168.1.253"
CAM_ARGS="--device 0 --names overhead --fps 25 --label E02 --no-lock"

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

# where the video goes
OUT=""
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
  read -r -p "Record to the Pi's own card instead? Type y and press Enter (anything else stops): " ans
  [ "$ans" = "y" ] || exit 1
  OUT="$HOME/recordings"
fi
mkdir -p "$OUT" || { echo "Can't write to $OUT. Stopped."; exit 1; }
free_gb=$(df -BG --output=avail "$OUT" | tail -1 | tr -dc 0-9)
echo "Video goes to $OUT ($free_gb GB free)."
# space needed: about 3 GB an hour of video plus 1 GB to spare (session 1: 5.5 GB in 2 h); --hours sets it
hours=2
[ "$PLAN" = 1 ] && hours=5.5
prev=""
for a in "$@"; do [ "$prev" = "--hours" ] && hours="$a"; prev="$a"; done
need_gb=$(python3 -c "import math,sys; print(int(math.ceil(3*float(sys.argv[1])+1)))" "$hours" 2>/dev/null || echo 7)
if [ "${free_gb:-0}" -lt "$need_gb" ]; then
  echo "Only $free_gb GB free; a $hours-hour session needs about $need_gb GB. Free some space first. Stopped."
  exit 1
fi
if [ ! -f "$HOME/.config/mirlt/github_token" ]; then
  echo "NOTE: no GitHub token yet (docs/pi-run.md, 'GitHub token'): the session runs, but the upload at the end"
  echo "      is skipped; the files stay on the Pi."
fi
git -C "$REPO" pull --ff-only >/dev/null 2>&1 && echo "Code up to date." || echo "(git pull skipped: offline or local changes)"

if [ "$#" -gt 0 ]; then RUN_ARGS="$RUN_ARGS_DEFAULT $*"; else RUN_ARGS="$RUN_ARGS_DEFAULT"; fi

tmux new-session -d -s e02 -n rec "cd '$REPO' && python3 tools/pi/camrec.py $CAM_ARGS --out '$OUT'; echo; read -r -p 'Recorder stopped. Press Enter to close this window.'"
echo "Recorder starting (8 s) ..."
sleep 8
if [ "$PLAN" = 1 ]; then
  tmux new-window -t e02 -n run "cd '$REPO' && bash tools/pi/session_plan.sh '$OUT'; echo; read -r -p 'Plan done. Press Enter to close this window.'"
else
  tmux new-window -t e02 -n run "cd '$REPO' && python3 tools/experiment/e02.py --on-pi $RUN_ARGS; echo; echo '=== Runner finished: stopping the recorder and uploading ==='; python3 tools/pi/upload_session.py --recordings '$OUT' --stop-recorder; echo; read -r -p 'All done. Press Enter to close this window.'"
fi
tmux select-window -t e02:run
echo
echo "Started. Watch it and answer its questions on any phone or computer on the WiFi:"
echo "    http://$(hostname).local:8090"
echo "Opening the runner's window now. To leave it running and close this terminal: Ctrl+B, then D."
sleep 2
tmux attach -t e02
