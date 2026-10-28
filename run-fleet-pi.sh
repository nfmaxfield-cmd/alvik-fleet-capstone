#!/usr/bin/env bash
# Phase 0 on the Pi (2026-10-27): the fleet planner (tools/fleet/run_plan.py) on the real robots, with camera fixes
# from the overhead camera through the camera bridge (tools/fleet/cam_bridge.py). Plan: docs/session-plans/2026-10-27.md
#
# ONE ADDRESS for everything (2026-10-28): http://100.122.222.75:8091 in the laptop's or phone's browser (the start
# page, always on). Tabs: Live lab (an E02 session's live page; a DOE loop's or Monday mission's own screen and Stop
# while one runs) | Mission Control (opened from there on demand: the same as `mc` below) | Start/Stop.
#
# In a terminal ON THE PI (SSH from the laptop), one of:
#     ./run-fleet-pi.sh setup              camera setup page (after moving a camera): overhead corners, side picture,
#                                          the lights' flash check; http://100.122.222.75:8096. Stop the recorder first
#     ./run-fleet-pi.sh rec                start the recorder only (its picture: http://<the Pi>:8000 in a browser)
#     ./run-fleet-pi.sh check              every robot's firmware and battery (nothing moves)
#     ./run-fleet-pi.sh turn Center        Center turns 90 deg once (repeat it 4 times: it should face the same way)
#     ./run-fleet-pi.sh calibrate          every robot on the board calibrates (about 65 s; needs room round each)
#     ./run-fleet-pi.sh one [Left]         test 1: one robot drives a square twice (put it on spot A first)
#     ./run-fleet-pi.sh cross              test 2: Left and Right cross paths 4 times (on A and B first)
#     ./run-fleet-pi.sh watch              brick watch only, robots standing on A and B: wave a hand, put a brick
#                                          down, take it away; it prints what it would report (nothing is sent)
#     ./run-fleet-pi.sh brick              hand-and-brick check: test 2 with the brick watch on
#     ./run-fleet-pi.sh stop               stop the recorder and upload today's Phase 0 logs and video
# The remote ladder (nobody at the board; robots left anywhere on it, 30 cm in, 40 cm apart): cautious to bold,
# each step only after the one before passed; Ctrl+C stops every robot (tools/fleet/ladder.py):
#     ./run-fleet-pi.sh ladder             L0 status .. L5 cross, from the first step not passed yet; stops at a FAIL
#     ./run-fleet-pi.sh ladder pause       the same, waiting for Enter before each step
#     ./run-fleet-pi.sh L3                 one step (L0 status, L1 calibrate, L2 turns, L3 straight, L4 square,
#                                          L5 cross, L6 brick: `L6 attended`, only with someone at the board)
#     ./run-fleet-pi.sh turncheck Center   the camera-measured turn check alone (replaces `turn` and the ruler)
#     ./run-fleet-pi.sh ladder-status      what has passed, where each robot is
# The DOE loop (unattended, tools/fleet/doe_loop.py): L0 + L1, then turn checks and straight out-and-backs at 5-13 cm/s
# one robot at a time until every robot is out or 4 h; runs in tmux (session doe), so SSH can be closed:
#     ./run-fleet-pi.sh doe [hours] [full-video]   start it (full-video: the recorder keeps its whole video too)
#     ./run-fleet-pi.sh doe-status         drives so far, battery, who is out
#     ./run-fleet-pi.sh doe-stop           stop it (every robot stops), then it uploads as when it finishes
# One system (2026-10-28): Monday's mission and Mission Control, the same program (tools/fleet/mission_run.py):
#     ./run-fleet-pi.sh monday [61x122] [Left,Right] [farm]   Monday's default mission on the table of that size (put
#                                          each robot at its own end first); runs in tmux "monday"; Ctrl+C stops every
#                                          robot; uploads at the end. farm: each camera tracked in its own process
#     ./run-fleet-pi.sh mc                 Mission Control on the Pi (also the Mission Control tab at :8091, which runs
#                                          this in tmux "mc"); its own address http://100.122.222.75:8095 still works
#     ./run-fleet-pi.sh mc-stop            close Mission Control opened from the tab (a run of its own stops first)
# The first test starts the recorder (both cameras, to the USB drive) in a tmux session called fleet; it keeps
# recording until `stop`. Each test starts the camera bridge, runs the plan, then prints the line check (GATE).
set -u
cd "$(dirname "$0")"
REPO="$(pwd)"
IPS="--robot 192.168.1.250 --robot 192.168.1.252 --robot 192.168.1.253"
# the overhead calibration and exposure saved by the camera setup page (./run-fleet-pi.sh setup, 2026-10-27);
# roam1 and the camera's automatic exposure when nothing was saved (setup_conf.py says which on the screen)
CALIB="$(python3 tools/pi/setup_conf.py calib)"
EXPO="$(python3 tools/pi/setup_conf.py exposure)"
EXPO_ARG=""; [ -n "$EXPO" ] && EXPO_ARG="--overhead-exposure $EXPO"
# the side camera's calibration (setup page, part 2; two-camera tracking 2026-10-27): with it, both cameras follow the
# robots (the recorder serves the side camera's pictures at 127.0.0.1:8001); without it, the overhead alone
SIDECAL="$(python3 tools/pi/setup_conf.py side)"
SIDE_ARG=""; [ -n "$SIDECAL" ] && SIDE_ARG="--side-calib $SIDECAL --side-stream 127.0.0.1:8001"
DAY=$(date +%Y%m%d)
# the robots' edge guard (both cues) and the bare body, as e02.py's SAFETY_SETTINGS: safe_edge is not saved on the
# robot, so every power-on starts with it OFF. Sent to every robot before each test.
SAFE=(--send "set safe_edge 3" --send "set edge_dir 0" --send "set edge_thr 150" --send "set edge_n 3"
      --send "set edge_bot_cm 11" --send "set bumper_cm 0")
WHAT="${1:-}"; shift || true

# the ladder's connection: the robots, the recorder's pictures, the calibration (the simulated board in dry runs:
# FLEET_DRY=1, LADDER_DRY_ARGS e.g. "--turn-err Center=18 --seed 7")
ladder() {
  if [ "$DRY" = 1 ]; then
    python3 tools/fleet/ladder.py "$@" --dry ${LADDER_DRY_ARGS:-}
  else
    start_recorder
    # cores (the e02 runner's plan, run-e02-pi.sh): the recorder on cores 0-1, the ladder and its tracking on 2-3
    local PIN=""
    if [ "$(nproc)" -ge 4 ] && command -v taskset >/dev/null; then PIN="taskset -c 2,3"; fi
    $PIN python3 tools/fleet/ladder.py "$@" $IPS --stream 127.0.0.1:8000 --calib "$CALIB" $SIDE_ARG
  fi
}

rec_running() { tmux has-session -t fleet 2>/dev/null && tmux list-windows -t fleet -F '#W' | grep -qx rec; }

# FLEET_DRY=1 ./run-fleet-pi.sh one|cross|brick: the same steps against the simulator (tools/sim/fleet_sim.py serves a
# simulated overhead camera); no recorder, no robots. How the launcher was tested.
DRY="${FLEET_DRY:-0}"
start_recorder() {
  if [ "$DRY" = 1 ]; then tmux has-session -t fleet 2>/dev/null || tmux new-session -d -s fleet -n idle "sleep 100000"; return 0; fi
  if rec_running; then echo "Recorder already running (tmux session fleet)."; return 0; fi
  if tmux has-session -t e02 2>/dev/null; then
    echo "An E02 session is running on this Pi (tmux e02). Stop it first. Stopped."; exit 1
  fi
  OD="$(python3 tools/pi/camnames.py --dev overhead)" || { echo "Stopped: no overhead camera found by its name."; exit 1; }
  SD="$(python3 tools/pi/camnames.py --dev side 2>/dev/null)" || SD=""
  OUT=""
  for m in /media/"$USER"/*; do
    [ -d "$m" ] && [ -w "$m" ] || continue
    OUT="$m/recordings"; break
  done
  [ -z "$OUT" ] && { echo "No USB drive found: recording to the Pi's card (~/recordings)."; OUT="$HOME/recordings"; }
  mkdir -p "$OUT"
  echo "$OUT" > /tmp/fleet_rec_out
  if [ -n "$SD" ] && [ "$SD" != "$OD" ]; then
    CAMS="--device $OD,$SD --names overhead,side"; REC_CORES="--cores 0,1"; echo "Recording both cameras (overhead /dev/video$OD, side /dev/video$SD) to $OUT"
  else
    CAMS="--device $OD --names overhead"; REC_CORES="--cores 1"; echo "Recording the overhead camera (/dev/video$OD) to $OUT (no side camera found)"
  fi
  python3 tools/pi/launcher.py --handoff >/dev/null 2>&1 || true
  # REC_EXTRA: the DOE loop's --no-record (pictures served, no video saved: the loop saves its own frames)
  # the side camera on automatic exposure (Nick, Q22: keep automatic; locked, it went dark at 13:01 and again later
  # as the room's light changed; the trackers follow the exposure's drift and brighten a dark side picture);
  # --shm: each camera's grey picture in shared memory for the camera bridge's workers (cam_bridge.py --farm)
  tmux new-session -d -s fleet -n rec "cd '$REPO' && python3 tools/pi/camrec.py $CAMS --fps 25 --label P0 --no-lock $EXPO_ARG --side-exposure auto --shm $REC_CORES ${REC_EXTRA:-} --out '$OUT'; read -r -p 'Recorder stopped. Press Enter.'"
  echo "Recorder starting (8 s) ..."; sleep 8
}

run_test() {   # run_test NAME MAP [bridge options]
  local name="$1" map="$2"; shift 2
  start_recorder
  local sess="logs/$(date +%Y%m%d-%H%M%S)-p0-$name"
  mkdir -p "$sess"
  # --place-tol-cm 4 (chosen: a robot put down by tape measure; further off, run_plan stops before anything moves
  # and says how far. No DOE: a placement check). No --yes on the robots: --yes would start a robot from anywhere.
  local cal="$CALIB" stream="$SIDE_ARG" rp="$IPS --place-tol-cm 4"
  if [ "$DRY" = 1 ]; then
    sess="$sess-dryrun"; mkdir -p "$sess"
    cal="$REPO/$sess/sim/table_camera.json"; stream="--stream 127.0.0.1:8765 --wait-calib-s 30"
    rp="--dry-run --yes --sim-arg=--fix-hz=0 --sim-arg=--serve-live=8765 --sim-arg=--live-calib-out=$cal"
    [ "$name" = brick ] && rp="$rp --brick auto@15 --sim-arg=--live-brick"
  fi
  tmux kill-window -t fleet:bridge 2>/dev/null || true
  tmux new-window -d -t fleet -n bridge "cd '$REPO' && python3 tools/fleet/cam_bridge.py --map '$map' --calib $cal $stream --out '$sess/bridge' $* 2>&1 | tee '$sess/bridge.txt'"
  echo "Camera bridge starting (it learns the empty floor round the robots: keep hands out of the picture) ..."
  sleep 5
  echo "=== Test $name: the robots start now. To stop them at any moment: press Ctrl+C ==="
  python3 tools/fleet/run_plan.py --map "$map" --session "$sess" --fixes udp --poses-to 127.0.0.1:5011 "${SAFE[@]}" $rp
  tmux send-keys -t fleet:bridge C-c 2>/dev/null || true
  sleep 3
  tmux kill-window -t fleet:bridge 2>/dev/null || true
  echo; echo "=== Line check (camera spots against the planned lines) ==="
  TR=""; [ "$DRY" = 1 ] && TR="--truth $sess/sim/truth.csv"
  python3 tools/fleet/line_check.py "$sess" "$sess/bridge" --png "$sess/line_check.png" $TR
  echo "Saved in $sess"
}

case "$WHAT" in
  ladder)
    case "${1:-}" in
      pause) ladder run --pause ;;
      from) ladder run --from "${2:-L0}" ;;
      *) ladder run ;;
    esac ;;
  L0|L1|L2|L3|L4|L5|L6|l0|l1|l2|l3|l4|l5|l6)
    A=""; [ "${1:-}" = attended ] && A="--attended"
    [ "${1:-}" = ignore ] && A="--ignore ${2:-}"     # L0 ignore Center: heard on the WiFi, not on the board
    ladder step "$WHAT" $A ;;
  turncheck)
    ladder turncheck "${1:-Center}" ;;
  doe)
    HRS=4; FULL=0
    for x in "$@"; do case "$x" in full-video) FULL=1 ;; [0-9]*) HRS="$x" ;; esac; done
    if tmux has-session -t doe 2>/dev/null; then
      echo "A DOE loop is already running (tmux session doe). Watch it: tmux capture-pane -p -t doe | tail -40"
      echo "Stop it first if you want a new one: ./run-fleet-pi.sh doe-stop"; exit 1
    fi
    for s in ladder ladder2; do tmux has-session -t $s 2>/dev/null && { echo "A ladder is running (tmux session $s): stop it first (tmux kill-session -t $s)."; exit 1; }; done
    [ "$DRY" = 1 ] || { [ "$FULL" = 1 ] || REC_EXTRA="--no-record"; start_recorder; }
    tmux new-session -d -s doe -x 200 -y 50 "cd '$REPO' && FLEET_DRY=$DRY DOE_DRY_ARGS='${DOE_DRY_ARGS:-}' ./run-fleet-pi.sh doe-run $HRS; FLEET_DRY=$DRY ./run-fleet-pi.sh doe-finish; read -r -p 'DOE loop finished. Press Enter.'"
    # the loop's whole screen (run_plan's lines too) to a file; doe-finish moves it into the DOE folder (screen.txt)
    tmux pipe-pane -t doe "cat >> '$REPO/logs/doe-screen.txt'" 2>/dev/null || true
    echo "DOE loop started in tmux session 'doe' (time cap $HRS h; full video: $([ $FULL = 1 ] && echo yes || echo no))."
    echo "You can close this window: it keeps running on the Pi."
    echo "Watch it:        tmux capture-pane -p -t doe | tail -40"
    echo "Or live:         tmux attach -t doe        (leave it again: Ctrl+B, then D)"
    echo "Pictures:        http://100.122.222.75:8000 (overhead), http://100.122.222.75:8001 (side)"
    echo "Progress:        ./run-fleet-pi.sh doe-status"
    echo "Stop it:         ./run-fleet-pi.sh doe-stop" ;;
  doe-run)          # (inside the tmux session doe)
    if [ "$DRY" = 1 ]; then
      python3 tools/fleet/doe_loop.py run --hours "${1:-4}" --dry ${DOE_DRY_ARGS:-}
    else
      PIN=""; if [ "$(nproc)" -ge 4 ] && command -v taskset >/dev/null; then PIN="taskset -c 2,3"; fi
      $PIN python3 tools/fleet/doe_loop.py run --hours "${1:-4}" $IPS --stream 127.0.0.1:8000 --calib "$CALIB" $SIDE_ARG
    fi ;;
  doe-finish)       # (inside the tmux session doe, after the loop): recorder stopped, logs + frames uploaded
    D="$(ls -td logs/*-doe 2>/dev/null | head -1)"
    if [ -f logs/doe-screen.txt ]; then
      DD="$D"; [ "$DRY" = 1 ] && DD="$(ls -td logs/*-doe-dryrun 2>/dev/null | head -1)"
      [ -n "$DD" ] && mv logs/doe-screen.txt "$DD/screen.txt"
    fi
    [ "$DRY" = 1 ] && { echo "(dry run: no recorder to stop, nothing uploaded; folder $(ls -td logs/*-doe-dryrun | head -1))"; exit 0; }
    OUT="$(cat /tmp/fleet_rec_out 2>/dev/null || echo "$HOME/recordings")"
    if rec_running; then tmux send-keys -t fleet:rec C-c; echo "Stopping the recorder ..."; sleep 6; fi
    [ -z "$D" ] && { echo "No DOE folder found."; exit 1; }
    python3 tools/pi/upload_session.py --recordings "$OUT" --session "$D" --tag "doe-$(basename "$D")" --unique-tag \
      --exclude frames --recording-dir "$D/frames" || echo "Upload failed: the files stay in $D (run ./run-fleet-pi.sh doe-finish again)" ;;
  doe-status)
    D=""; [ "$DRY" = 1 ] && D="--dry"
    python3 tools/fleet/doe_loop.py status $D
    tmux has-session -t doe 2>/dev/null && { echo "--- the loop's screen (last 15 lines) ---"; tmux capture-pane -p -t doe | grep -v '^$' | tail -15; } || echo "(no DOE loop running)" ;;
  doe-stop)
    if tmux has-session -t doe 2>/dev/null; then
      tmux send-keys -t doe C-c; echo "Sent Ctrl+C to the DOE loop: every robot stops; it then uploads (watch: tmux capture-pane -p -t doe | tail -20)."
    else echo "No DOE loop running."; fi ;;
  monday)
    TABLE="61x122"; ROB="Left,Right"; EXTRA=""
    for x in "$@"; do case "$x" in [0-9]*x[0-9]*) TABLE="$x" ;; *,*) ROB="$x" ;; farm) EXTRA="$EXTRA --farm --rings" ;; esac; done
    if [ "$DRY" = 1 ]; then
      python3 tools/fleet/mission_run.py --table "$TABLE" --robots "$ROB" --dry ${MONDAY_DRY_ARGS:-}
    else
      start_recorder
      OUT="$(cat /tmp/fleet_rec_out 2>/dev/null || echo "$HOME/recordings")"
      # cores: the recorder on 0-1, the camera bridge's workers on 1-2 (--farm), the bridge and run_plan on 3
      PIN=""; if [ "$(nproc)" -ge 4 ] && command -v taskset >/dev/null; then PIN="taskset -c 2,3"; fi
      # in tmux (session "monday"), so a dropped WiFi or a closed window does not stop the run half-way
      tmux kill-session -t monday 2>/dev/null || true
      tmux new-session -s monday "cd '$REPO' && $PIN python3 tools/fleet/mission_run.py --table '$TABLE' --robots '$ROB' --calib '$CALIB' ${SIDECAL:+--side-calib '$SIDECAL'} --upload --recordings '$OUT' $EXTRA; read -r -p 'Mission ended. Press Enter.'"
      echo "If the window closed: tmux attach -t monday   (Ctrl+C in it stops every robot)"
    fi ;;
  mc)
    if python3 -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8095/api/run', timeout=2)" 2>/dev/null; then
      echo "Mission Control is already open: http://100.122.222.75:8091 (Mission Control tab) or http://100.122.222.75:8095"; exit 0
    fi
    # the recorder (its pictures for the robots' runs); in a subshell, so that an E02 session holding the cameras
    # leaves Mission Control open for planning and the simulator instead of stopping here
    if [ "$DRY" != 1 ]; then
      ( start_recorder ) || echo "(No recorder: Mission Control opens for planning and the simulator only; robot runs need the cameras.)"
    fi
    # the addresses the page may make changes from (its Host check): Tailscale, the Pi's name on the WiFi, its WiFi number
    HOSTS="--allow-host 100.122.222.75 --allow-host $(hostname).local"
    for ip in $(hostname -I 2>/dev/null); do case "$ip" in *.*.*.*) HOSTS="$HOSTS --allow-host $ip" ;; esac; done
    echo "Mission Control: http://100.122.222.75:8091 (Mission Control tab), or http://100.122.222.75:8095 (Ctrl+C here stops it)"
    python3 tools/fleet/mission_control.py --dir logs/mission-control --listen 0.0.0.0 $HOSTS ;;
  mc-stop)
    if tmux has-session -t mc 2>/dev/null; then tmux send-keys -t mc C-c; echo "Mission Control is closing."
    else echo "Mission Control was not opened from the tab (tmux mc): if it runs in a terminal, Ctrl+C there."; fi ;;
  ladder-status)   # (no recorder needed)
    D=""; [ "$DRY" = 1 ] && D="--dry"
    python3 tools/fleet/ladder.py status $D ;;
  ladder-reset)
    D=""; [ "$DRY" = 1 ] && D="--dry"
    python3 tools/fleet/ladder.py reset $D ;;
  setup)
    if rec_running; then
      echo "The recorder is running (tmux session fleet) and holds the cameras. Stop it first with:"
      echo "    tmux kill-window -t fleet:rec"
      echo "then run ./run-fleet-pi.sh setup again."; exit 1
    fi
    if tmux has-session -t e02 2>/dev/null; then echo "An E02 session is running on this Pi (tmux e02). Stop it first. Stopped."; exit 1; fi
    python3 tools/pi/camera_setup.py "$@" ;;
  rec)
    start_recorder
    echo "Recorder running. Its picture: http://100.122.222.75:8000 (Tailscale) in the laptop's browser" ;;
  check)
    python3 tools/fleet/robot_cmd.py $IPS --wait-s 3 status ;;
  turn)
    R="${1:-Center}"
    python3 tools/fleet/robot_cmd.py $IPS --to "$R" --wait-s 6 "turn 90" ;;
  calibrate)
    python3 tools/fleet/robot_cmd.py $IPS --wait-s 75 calibrate ;;
  one)
    R="${1:-Left}"
    M="logs/p0-one-$R.json"
    python3 -c "import json,sys; m=json.load(open('tools/fleet/examples/phase0_one.json')); m['robots'][0]['name']=sys.argv[1]; m['tasks']={sys.argv[1]: m['tasks']['Left']}; json.dump(m, open(sys.argv[2],'w'), indent=1)" "$R" "$M"
    run_test "one-$R" "$M" ;;
  cross)
    run_test cross tools/fleet/examples/phase0_cross.json ;;
  watch)
    start_recorder
    sess="logs/$(date +%Y%m%d-%H%M%S)-p0-watch"; mkdir -p "$sess"
    echo "=== Brick watch only (nothing is sent to the robots). Stop with Ctrl+C ==="
    python3 tools/fleet/cam_bridge.py --map tools/fleet/examples/phase0_cross.json --calib $CALIB --observe --brick-watch \
      --out "$sess/bridge" 2>&1 | tee "$sess/bridge.txt" ;;
  brick)
    run_test brick tools/fleet/examples/phase0_cross.json --brick-watch ;;
  stop)
    OUT="$(cat /tmp/fleet_rec_out 2>/dev/null || echo "$HOME/recordings")"
    tmux kill-window -t fleet:bridge 2>/dev/null || true
    for d in $(ls -td logs/$DAY-*-p0-* logs/$DAY-ladder 2>/dev/null); do
      python3 tools/pi/upload_session.py --recordings "$OUT" --logs-only --session "$d" --tag "p0-$(basename "$d")" || true
    done
    python3 tools/pi/upload_session.py --recordings "$OUT" --stop-recorder --no-logs --tag "p0-$DAY-recording" || true
    tmux kill-session -t fleet 2>/dev/null || true ;;
  *)
    sed -n '2,43p' "$0"; exit 1 ;;
esac
