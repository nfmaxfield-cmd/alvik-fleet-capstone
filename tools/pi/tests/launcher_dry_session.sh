#!/usr/bin/env bash
# Stand-in for run-e02-pi.sh when testing the start page (tools/pi/launcher.py --script ...): the same tmux layout
# (session e02, windows rec and run), the same chain in the run window (runner; then the upload, which stops the
# recorder), but the runner drives the simulator (--dry-run), the recorder is a stand-in that writes a sparse video
# file, and the upload's GitHub calls are stubbed (tests/dry/upload_session.py). Takes the same arguments as run-e02-pi.sh.
# Environment: DRY_REC (the recordings folder), LAUNCHER_PORT (default 8091), MIRLT_V4L_SYSFS (stand-in cameras).
set -u
cd "$(dirname "$0")/../../.."
REPO="$(pwd)"
SIDE=0; PROD=0; ARGS=(); CLASSIC=0
for a in "$@"; do
  case "$a" in
    --with-side) SIDE=1;;
    --production) PROD=1;;
    --classic-setup) CLASSIC=1; ARGS+=("$a");;
    *) ARGS+=("$a");;
  esac
done
echo "=== E02 dry session (start-page test) ==="
OD="$(python3 tools/pi/camnames.py --dev overhead)" || { echo "Stopped: the overhead camera was not found by its name."; exit 1; }
echo "Overhead camera: /dev/video$OD (found by its name)."
if [ "$SIDE" = 1 ]; then
  SD="$(python3 tools/pi/camnames.py --dev side)" || { echo "Stopped: the side camera was not found by its name."; exit 1; }
  echo "Side camera: /dev/video$SD."
fi
if tmux has-session -t e02 2>/dev/null; then
  echo "A session is already running on this Pi. To see it:  tmux attach -t e02"; exit 1
fi
python3 tools/pi/launcher.py --handoff --port "${LAUNCHER_PORT:-8091}" >/dev/null 2>&1 || true
REC="${DRY_REC:-/tmp/dry-rec}"
mkdir -p "$REC"
EXTRA="--skip-naming --skip-find-check --hours 0.5"
[ "$CLASSIC" = 1 ] && EXTRA="$EXTRA --dry-true-calib"
RUN="python3 tools/experiment/e02.py --dry-run --web-ask --view-open $EXTRA ${ARGS[*]}"
if [ "$PROD" = 1 ]; then
  tmux new-session -d -s e02 -n run "cd '$REPO' && $RUN; echo; echo '=== Runner finished: uploading the logs and frames ==='; python3 tools/pi/tests/dry/upload_session.py --recordings '$REC' --logs-only; echo; read -r -p 'All done. Press Enter to close this window.'"
  echo "Started (production, dry)."; exit 0
fi
tmux new-session -d -s e02 -n rec "cd '$REPO' && python3 tools/pi/tests/dry/camrec.py '$REC' $SIDE; echo; read -r -p 'Recorder stopped. Press Enter to close this window.'"
echo "Recorder starting (2 s) ..."
sleep 2
tmux new-window -t e02 -n run "cd '$REPO' && $RUN; echo; echo '=== Runner finished: stopping the recorder and uploading ==='; python3 tools/pi/tests/dry/upload_session.py --recordings '$REC' --stop-recorder; echo; read -r -p 'All done. Press Enter to close this window.'"
echo "Started (dry)."
