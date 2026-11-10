#!/usr/bin/env bash
# (2026-11-10 e13) calib, freeroam, doe, visit and the missions close a session whose run has ended (its window waiting
# at "Press Enter", found by the session's name: session_ended) before starting, instead of refusing the new run.
# Phase 0 on the Pi (2026-10-27): the fleet planner (tools/fleet/run_plan.py) on the real robots, with camera fixes
# from the overhead camera through the camera bridge (tools/fleet/cam_bridge.py). Plan: docs/session-plans/2026-10-27.md
#
# ONE ADDRESS for everything (2026-10-28): http://<the Pi>:8091 in the laptop's or phone's browser (the start
# page, always on). Tabs: Live lab (an E02 session's live page; a DOE loop's or Monday mission's own screen and Stop
# while one runs) | Mission Control (opened from there on demand: the same as `mc` below) | Start/Stop.
#
# In a terminal ON THE PI (SSH from the laptop), one of:
#     ./run-fleet-pi.sh setup              camera setup page (after moving a camera): overhead corners, side picture,
#                                          the lights' flash check; http://<the Pi>:8096. Stop the recorder first
#     ./run-fleet-pi.sh rec                start the recorder only (its picture: http://<the Pi>:8000 in a browser)
#     ./run-fleet-pi.sh check              every robot's firmware and battery (nothing moves)
#     ./run-fleet-pi.sh turn Center        Center turns 90 deg once (repeat it 4 times: it should face the same way)
#     ./run-fleet-pi.sh calibrate          every robot on the board calibrates, one at a time (about 65 s each), or
#                                          keeps its calibration from earlier today on the same power-on (boot ID);
#                                          the camera checks its spin room (12 cm from every edge, 21 cm from the
#                                          others) and points its roll at open space. `calibrate full`: always a
#                                          full calibration. `calibrate blind`: the old way (no camera, all at once)
#     ./run-fleet-pi.sh one [Left]         test 1: one robot drives a square twice (put it on spot A first)
#     ./run-fleet-pi.sh cross              test 2: Left and Right cross paths 4 times (on A and B first)
#     ./run-fleet-pi.sh watch              brick watch only, robots standing on A and B: wave a hand, put a brick
#                                          down, take it away; it prints what it would report (nothing is sent)
#     ./run-fleet-pi.sh brick              hand-and-brick check: test 2 with the brick watch on
#     ./run-fleet-pi.sh stop               stop the recorder and upload today's Phase 0 logs and video
# The remote ladder (nobody at the board; robots left anywhere on it, 12 cm in and 21 cm apart to calibrate, 30 cm in
# and 40 cm apart to save the DOE loop moving them in first): cautious to bold,
# each step only after the one before passed; Ctrl+C stops every robot (tools/fleet/ladder.py):
#     ./run-fleet-pi.sh ladder             L0 status .. L5 cross, from the first step not passed yet; stops at a FAIL
#     ./run-fleet-pi.sh ladder pause       the same, waiting for Enter before each step
#     ./run-fleet-pi.sh L3                 one step (L0 status, L1 calibrate, L2 turns, L3 straight, L4 square,
#                                          L5 cross, L6 brick: `L6 attended`, only with someone at the board)
#     ./run-fleet-pi.sh turncheck Center   the camera-measured turn check alone (replaces `turn` and the ruler)
#     ./run-fleet-pi.sh ladder-status      what has passed, where each robot is
# The DOE loop (unattended, tools/fleet/doe_loop.py): L0 + L1, then turn checks and straight out-and-backs at 5-13 cm/s
# one robot at a time until every robot is out or 4 h; runs in tmux (session doe), so SSH can be closed:
#     ./run-fleet-pi.sh doe [hours] [full-video] [park] [low-battery]   (e08: keep-going is the default; park = old way)
#     (before e08:) ./run-fleet-pi.sh doe [hours] [full-video] [keep-going] [low-battery]   start it (full-video: the recorder keeps
#                                          its whole video too; keep-going: no robot is set aside by a turn check, crawl,
#                                          line check or one camera's evidence, only for safety: "would have parked" is
#                                          logged; low-battery: robots drive down to 5 % instead of 25 %, the loop's check
#                                          and every drive alike. CHARGE THE ROBOTS STRAIGHT AFTER A LOW-BATTERY RUN)
#     ./run-fleet-pi.sh doe-status         drives so far, battery, who is out
#     ./run-fleet-pi.sh doe-stop           stop it (every robot stops), then it uploads as when it finishes
#     ./run-fleet-pi.sh reupload-frames 20261029-074342-doe   send a DOE loop's frames to its release again (after a
#                                          failed upload; a zip over 1.9 GB goes in parts, parts already up are skipped)
# Home free roam (2026-11-05, tools/fleet/freeroam.py): Left, Center and Right moving at the same time, all the time,
# until each battery is at 10 % (then it parks by an edge and sleeps) or the time cap; L0 + L1 first as the DOE loop;
# runs in tmux (session freeroam): closing SSH or switching Tailscale accounts does not stop it; uploads at the end:
#     ./run-fleet-pi.sh freeroam [hours] [hold] [stopturn] [full-video]   start it (5 Nov 16:06: curves by default on
#                                          firmware 2026-11-05c, older robots stop-turn-drive; stopturn = everyone as before)
#     (was) ./run-fleet-pi.sh freeroam [hours] [hold] [full-video]   start it (default 4 h; hold: the old cautious halt when
#                                          the camera loses a robot instead of dead reckoning; full-video: the recorder
#                                          keeps its whole video too). CHARGE THE ROBOTS STRAIGHT AFTER IT
#     ./run-fleet-pi.sh freeroam-status    minutes so far, each robot's battery and moves, reflex events, full stops
#     ./run-fleet-pi.sh freeroam-stop      stop it (every robot stops), then it uploads as when it finishes
#     FLEET_DRY=1 ./run-fleet-pi.sh freeroam 0.5    the same against the simulator (no robots, no camera)
# One system (2026-10-28): Monday's mission and Mission Control, the same program (tools/fleet/mission_run.py):
#     ./run-fleet-pi.sh monday [61x122] [Left,Right] [farm]   Monday's default mission on the table of that size (put
#                                          each robot at its own end first; a facing check by their lights measures
#                                          which way each faces; one that is off is turned on the spot, or driven to its
#                                          spot if over 3 cm off (5 cm side camera alone), then checked again: straighten.py;
#                                          it stops only if a robot cannot be found or is still off after two
#                                          corrections); runs in tmux "monday"; Ctrl+C stops every
#                                          robot; uploads at the end. farm: each camera tracked in its own process
#     ./run-fleet-pi.sh practice mow|circle|chicken|dance [61x122]   ONE block of Monday's mission alone (tmux
#                                          "practice"; tools/fleet/examples/practice/*.json); as `monday` otherwise
#     ./run-fleet-pi.sh stopall            STOP EVERYTHING: Ctrl+C to every fleet run in tmux, Mission Control's Stop,
#                                          and `stop` straight to every robot (works while a run holds the robots): the
#                                          site's robots, a broadcast to the whole WiFi, every robot heard (2026-11-03).
#                                          `stopall listed`: the old way, only the listed addresses
#     ./run-fleet-pi.sh settings [Left]    bench: send gyro_scale 1.075 and wheel_scale 0.979, read them back
#     ./run-fleet-pi.sh dance Left         bench: the dance alone where it stands, about 8 s (firmware 2026-10-28a)
#     ./run-fleet-pi.sh mc                 Mission Control on the Pi, in tmux session "mc" (2026-11-03: it keeps running
#                                          when this window closes); prints http://<this Pi>:8095. `mc here`: the old
#                                          way, in this window. See it: tmux attach -t mc (leave: Ctrl+B, then D)
#     ./run-fleet-pi.sh mc-stop            close Mission Control (a run of its own stops first)
#     ./run-fleet-pi.sh address            this Pi's addresses (Tailscale, WiFi, name) and the one the screens show
#     ./run-fleet-pi.sh school-setup       (2026-11-04) the school Pi in one go: installs what is missing (tmux,
#                                          v4l-utils, Python packages; sudo), the camera's focus, the school site,
#                                          then the self-check (tools/pi/school_setup.sh)
#     ./run-fleet-pi.sh calib [dry]        (2026-11-04) the cautious calibration run, zones 1-2, in tmux "calib"
#                                          (tools/fleet/calib_run.py); Mission Control's Production screen starts it too
#     ./run-fleet-pi.sh calib-upload [folder]   the newest calibration run's folder uploaded (a release of its own)
#     ./run-fleet-pi.sh selfcheck          a new Pi's setup tested: packages, OpenCV, tags, cameras, repo, token,
#                                          Tailscale, clock, site, disk: one PASS / WARN / FAIL line each
# Sites (2026-11-02, tools/fleet/sites.py): home is the default and unchanged; a school site has its own robots, cameras,
# calibration folder and Mission Control folder:
#     ./run-fleet-pi.sh site               which site is in use, and the list
#     ./run-fleet-pi.sh site school-conference    use that site from now on (every command and Mission Control)
#     ./run-fleet-pi.sh camcheck           the camera check-out (size, rate, sharpness, exposure, coverage, tag size)
# Free space (2026-10-28: the 21:34 DOE loop filled the card at 22:03). Frames go to the USB drive; every long run
# checks the card and the drive before it starts and between loops, and ends cleanly before the card fills:
#     ./run-fleet-pi.sh space              free space on the card and the USB drive, and the biggest folders
#     ./run-fleet-pi.sh uploads            the last 10 sessions: ✓ uploaded / ✗ not, and their size here (quick)
#     ./run-fleet-pi.sh clean-uploaded     lists the sessions GitHub has (quick: the release has every file the upload
#                                          sent; seconds, low priority); deletes nothing. Then `... clean-uploaded --yes`
#                                          deletes exactly those (--frames-only: keep every logs folder; --thorough: the
#                                          old exact-size check, ~1 min a GB; --force: even while a robot run is going)
#     After every upload (DOE loop, free roam, mission, calibration) the session's big files (video, frames) are
#     deleted once GitHub has them; logs, CSVs and report stay. Off: MIRLT_AUTO_CLEAN=0 (on at home, off at school)
# The first test starts the recorder (both cameras, to the USB drive) in a tmux session called fleet; it keeps
# recording until `stop`. Each test starts the camera bridge, runs the plan, then prints the line check (GATE).
set -u
cd "$(dirname "$0")"
REPO="$(pwd)"
IPS="--robot 192.168.1.250 --robot 192.168.1.252 --robot 192.168.1.253"
# the site in use (2026-11-02, tools/fleet/sites.py): home (the default) gives exactly the line above; a school site
# its own robots (none listed: found on the WiFi by their hello), its own calibration folder (MIRLT_CONF_DIR), picture
# size and Mission Control folder. Change it with: ./run-fleet-pi.sh site <name>
# A site file that does not load STOPS here with the file, the line and what is wrong (risk review 2 Nov, #9: a typo
# used to fall back to home's robots and picture size in silence). Two exceptions: `site` (to switch to a good one) and
# `stopall` (the emergency stop never waits on a file: it still broadcasts `stop`).
WHAT0="${1:-}"
# (2026-11-04) the school Pi's setup runs before anything else here needs the site or the Python packages
if [ "$WHAT0" = school-setup ]; then shift; exec bash tools/pi/school_setup.sh "$@"; fi
if SITE_ENV="$(python3 tools/fleet/sites.py env 2>&1)"; then eval "$SITE_ENV"; IPS="$MIRLT_ROBOT_ARGS"
else
  case "$WHAT0" in
    site|stopall|selfcheck|"") echo "WARNING: $SITE_ENV" >&2 ;;
    *) echo "$SITE_ENV" >&2; echo "STOPPED: the site file must load first (nothing was started)." >&2; exit 1 ;;
  esac
fi
CAPTURE="${MIRLT_CAPTURE:---width 640 --height 480}"; CAPFPS="${MIRLT_CAPTURE_FPS:-25}"
# the Pi's own address for the screens (risk review 2 Nov, #3): detected on this Pi (Tailscale if up, else its WiFi
# address, else its name), never a hard-coded one (tools/fleet/pi_addr.py; a site may still pin one: use_site_address)
PIADDR="$(python3 tools/fleet/pi_addr.py 2>/dev/null)"; [ -z "$PIADDR" ] && PIADDR="$(hostname).local"
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
      --send "set edge_bot_cm 11" --send "set bumper_cm 0" --send "set tilt_stop_deg 5" --send "set tilt_ms 300")
# (2026-11-05h) the tilt stop too (Nick 20:46: a robot seen tilted at the board's edge): run_plan reads each back
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

# (e13, 10 Nov school: `calib` refused a new run because session "calib" still existed, its run long over and its
# window waiting at "Calibration run ended. Press Enter."; Nick saw only the old output 25 minutes later) a session
# whose run has ended: it exists, but the shell in its window has no child process left (the run's python3, or the
# launcher it ran, has exited; `read` is the shell's own). Found by the session's name and its pane's own process id,
# never by a process name pattern.
session_ended() {
  local s="$1" pid
  tmux has-session -t "$s" 2>/dev/null || return 1
  pid=$(tmux list-panes -t "$s" -F '#{pane_pid}' 2>/dev/null | head -1)
  [ -n "$pid" ] || return 1
  [ -z "$(pgrep -P "$pid" 2>/dev/null)" ]
}
close_if_ended() {     # closes the named session when its run has ended (said plainly); nothing otherwise
  if session_ended "$1"; then
    tmux kill-session -t "$1" 2>/dev/null || true
    echo "The previous run had ended; its window is closed (tmux session $1)."
  fi
}

# FLEET_DRY=1 ./run-fleet-pi.sh one|cross|brick: the same steps against the simulator (tools/sim/fleet_sim.py serves a
# simulated overhead camera); no recorder, no robots. How the launcher was tested.
DRY="${FLEET_DRY:-0}"
start_recorder() {
  if [ "$DRY" = 1 ]; then tmux has-session -t fleet 2>/dev/null || tmux new-session -d -s fleet -n idle "sleep 100000"; return 0; fi
  if rec_running; then
    # the window alone is not enough (16:24, 28 Oct: window open, no pictures from 8000, the DOE loop stopped at L0):
    # one picture from each camera within 3 s, else the recorder window is restarted
    local want_side=""; [ -n "$SIDECAL" ] && want_side="--side"
    if python3 tools/pi/rec_alive.py $want_side; then echo "Recorder already running (tmux session fleet), pictures coming."; return 0; fi
    echo "Restarting the recorder window (Ctrl+C to it, then a fresh start) ..."
    tmux send-keys -t fleet:rec C-c 2>/dev/null; sleep 4
    rec_running && tmux kill-window -t fleet:rec 2>/dev/null; sleep 1
  fi
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
  local REC_NEW="tmux new-session -d -s fleet -n rec"
  tmux has-session -t fleet 2>/dev/null && REC_NEW="tmux new-window -t fleet -n rec"
  $REC_NEW "cd '$REPO' && python3 tools/pi/camrec.py $CAMS $CAPTURE --fps $CAPFPS --label P0 --no-lock $EXPO_ARG --side-exposure auto --shm $REC_CORES ${REC_EXTRA:-} --out '$OUT'; read -r -p 'Recorder stopped. Press Enter.'"
  echo "Recorder starting (8 s) ..."; sleep 8
  local ws=""; [ -n "$SD" ] && [ "$SD" != "$OD" ] && ws="--side"
  python3 tools/pi/rec_alive.py $ws --timeout 6 || echo "WARNING: the recorder started, but pictures are still missing. Look at its window: tmux attach -t fleet (leave: Ctrl+B, then D)."
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
    HRS=4; FULL=0; KEEP="keep-going"; MINB=25     # (e08, Nick 7 Nov Q7) keep going is the default; "park" = the old way
    for x in "$@"; do case "$x" in full-video) FULL=1 ;; keep-going) KEEP="keep-going" ;; park|no-keep-going) KEEP="park" ;; low-battery) MINB=5 ;; [0-9]*) HRS="$x" ;; esac; done
    close_if_ended doe
    for s in ladder ladder2; do close_if_ended $s; done
    if tmux has-session -t doe 2>/dev/null; then
      echo "A DOE loop is already running (tmux session doe). Watch it: tmux capture-pane -p -t doe | tail -40"
      echo "Stop it first if you want a new one: ./run-fleet-pi.sh doe-stop"; exit 1
    fi
    for s in ladder ladder2; do tmux has-session -t $s 2>/dev/null && { echo "A ladder is running (tmux session $s): stop it first (tmux kill-session -t $s)."; exit 1; }; done
    # where the frames go and how much room there is; not started when the card is under its floor
    python3 tools/fleet/diskguard.py preflight || exit 1
    [ "$DRY" = 1 ] || { [ "$FULL" = 1 ] || REC_EXTRA="--no-record"; start_recorder; }
    tmux new-session -d -s doe -x 200 -y 50 "cd '$REPO' && export MIRLT_MIN_BATTERY=$MINB && FLEET_DRY=$DRY DOE_DRY_ARGS='${DOE_DRY_ARGS:-}' ./run-fleet-pi.sh doe-run $HRS $KEEP; FLEET_DRY=$DRY ./run-fleet-pi.sh doe-finish; read -r -p 'DOE loop finished. Press Enter.'"
    # the loop's whole screen (run_plan's lines too) to a file; doe-finish moves it into the DOE folder (screen.txt)
    tmux pipe-pane -t doe "cat >> '$REPO/logs/doe-screen.txt'" 2>/dev/null || true
    echo "DOE loop started in tmux session 'doe' (time cap $HRS h; robots set aside under $MINB% battery; full video: $([ $FULL = 1 ] && echo yes || echo no); keep-going: $([ "$KEEP" = keep-going ] && echo "yes, no robot is set aside except for safety" || echo "no (park)"))."
    [ "$MINB" -lt 25 ] && echo "LOW-BATTERY RUN: the robots drive down to $MINB%. Charge them straight after it."
    echo "You can close this window: it keeps running on the Pi."
    echo "Watch it:        tmux capture-pane -p -t doe | tail -40"
    echo "Or live:         tmux attach -t doe        (leave it again: Ctrl+B, then D)"
    echo "Pictures:        http://$PIADDR:8000 (overhead), http://$PIADDR:8001 (side)"
    echo "Progress:        ./run-fleet-pi.sh doe-status"
    echo "Stop it:         ./run-fleet-pi.sh doe-stop" ;;
  doe-run)          # (inside the tmux session doe)
    if [ "$DRY" = 1 ]; then
      KG="--keep-going"; [ "${2:-}" = park ] && KG="--no-keep-going"
      python3 tools/fleet/doe_loop.py run --hours "${1:-4}" --dry $KG ${DOE_DRY_ARGS:-}
    else
      PIN=""; if [ "$(nproc)" -ge 4 ] && command -v taskset >/dev/null; then PIN="taskset -c 2,3"; fi
      KG="--keep-going"; [ "${2:-}" = park ] && KG="--no-keep-going"
      $PIN python3 tools/fleet/doe_loop.py run --hours "${1:-4}" $KG $IPS --stream 127.0.0.1:8000 --calib "$CALIB" $SIDE_ARG
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
    # the frames: on the USB drive since 2026-10-28 (frames_where.txt in the folder says where), else $D/frames
    FR="$(python3 tools/fleet/diskguard.py where "$D")"
    python3 tools/pi/upload_session.py --recordings "$OUT" --session "$D" --tag "doe-$(basename "$D")" --unique-tag \
      --exclude frames --recording-dir "$FR" || echo "Upload failed: the files stay in $D and $FR (run ./run-fleet-pi.sh doe-finish again)" ;;
  reupload-frames)   # (2026-10-29) a session's frames again, into the release its logs went to; nothing deleted
    N="${1:-}"; [ -z "$N" ] && { echo "Which session? e.g. ./run-fleet-pi.sh reupload-frames 20261029-074342-doe"; exit 1; }
    D="logs/$N"; [ -d "$D" ] || { echo "No folder $D on the Pi."; exit 1; }
    FR="$(python3 tools/fleet/diskguard.py where "$D")"
    [ -d "$FR" ] || { echo "The frames folder $FR is not there (is the USB drive plugged in?)."; exit 1; }
    OUT="$(cat /tmp/fleet_rec_out 2>/dev/null || echo "$HOME/recordings")"
    TAG="${2:-doe-$N}"; case "$N" in *-mission*) TAG="${2:-mission-$N}" ;; esac
    echo "Uploading the frames in $FR to release $TAG (zipping first: a few minutes per GB) ..."
    python3 tools/pi/upload_session.py --recordings "$OUT" --session "$D" --tag "$TAG" --no-logs --recording-dir "$FR" \
      && echo "Done. (The Auto-clean lines above say what was freed; with MIRLT_AUTO_CLEAN=0 the frames stay until ./run-fleet-pi.sh clean-uploaded --yes.)" \
      || echo "Upload failed: run the same command again (parts already up are skipped)." ;;
  doe-status)
    D=""; [ "$DRY" = 1 ] && D="--dry"
    python3 tools/fleet/doe_loop.py status $D
    tmux has-session -t doe 2>/dev/null && { echo "--- the loop's screen (last 15 lines) ---"; tmux capture-pane -p -t doe | grep -v '^$' | tail -15; } || echo "(no DOE loop running)" ;;
  doe-stop)
    if tmux has-session -t doe 2>/dev/null; then
      tmux send-keys -t doe C-c; echo "Sent Ctrl+C to the DOE loop: every robot stops; it then uploads (watch: tmux capture-pane -p -t doe | tail -20)."
    else echo "No DOE loop running."; fi ;;
  freeroam)
    HRS=4; FULL=0; LOST=""; RULEENV=""
    # (e10) probe: robots may slowly prove ground the camera cannot see (off by default); no-discovery: no proof drive at
    # the start; typed-board: plan on the typed board, not the proven workspace; careful: only ground seen this run.
    # (e09) filter: the filter in charge of the poses (off by default); old-tracking: the tracking before e09 (no
    # lock-on, names by prediction); gyro-heading:
    # B3 on; no-brick-watch: the camera's "what changed" watch off.
    # (7 Nov, Nick) sleep-parked: a robot parked at 10 % goes to sleep until switched off and on (`sleep 0`); without it
    # a parked robot stays awake (the default since 11-05h).
    # (e08) attended: a person stays with the robots (the run may ask about a robot near an edge); old-rules: every rule
    # before e08 (fixed edges, 2 s pose rule, permanent keep-outs, out when silent / no heading, no tilt recovery, 5 deg)
    for x in "$@"; do case "$x" in full-video) FULL=1 ;; hold) LOST="$LOST --lost-mode hold" ;; stopturn) LOST="$LOST --motion stopturn" ;; curves) LOST="$LOST --motion curves" ;; attended|--attended) LOST="$LOST --attended" ;; old-rules) LOST="$LOST --edge-rule fixed --pose-rule wait2s --keepout-rule permanent --silent-out --heading-out --no-tilt-recovery"; RULEENV="MIRLT_TILT_RULE=old MIRLT_EDGE_RULE=fixed" ;; old-tracking) LOST="$LOST --old-tracking --pose-source blend" ;; filter) LOST="$LOST --pose-source ekf" ;; gyro-heading) LOST="$LOST --gyro-heading" ;; sleep-parked) LOST="$LOST --sleep-parked" ;; no-brick-watch) LOST="$LOST --no-brick-watch" ;; probe) LOST="$LOST --probe-ahead" ;; no-discovery) LOST="$LOST --no-discovery" ;; typed-board) LOST="$LOST --typed-board" ;; careful) LOST="$LOST --careful" ;; [0-9]*) HRS="$x" ;; esac; done
    for s in freeroam doe monday practice ladder ladder2 calib; do close_if_ended $s; done
    if tmux has-session -t freeroam 2>/dev/null; then
      echo "A free roam is already running (tmux session freeroam). Its progress: ./run-fleet-pi.sh freeroam-status"
      echo "Stop it first if you want a new one: ./run-fleet-pi.sh freeroam-stop"; exit 1
    fi
    for s in doe monday practice ladder ladder2 calib; do
      tmux has-session -t $s 2>/dev/null && { echo "A run is going in tmux session $s: stop it first (./run-fleet-pi.sh stopall). Stopped."; exit 1; }
    done
    python3 tools/fleet/diskguard.py preflight || exit 1
    [ "$DRY" = 1 ] || { [ "$FULL" = 1 ] || REC_EXTRA="--no-record"; start_recorder; }
    tmux new-session -d -s freeroam -x 200 -y 50 "cd '$REPO' && $RULEENV FLEET_DRY=$DRY ./run-fleet-pi.sh freeroam-run $HRS $LOST; FLEET_DRY=$DRY ./run-fleet-pi.sh freeroam-finish; read -r -p 'Free roam finished. Press Enter.'"
    tmux pipe-pane -t freeroam "cat >> '$REPO/logs/freeroam-screen.txt'" 2>/dev/null || true
    echo "Free roam started in tmux session 'freeroam' (time cap $HRS h; each robot parks at 10 % battery)."
    echo "You can close this window and switch Tailscale: it keeps running on the Pi."
    echo "Progress:        ./run-fleet-pi.sh freeroam-status"
    echo "Watch it live:   tmux attach -t freeroam        (leave it again: Ctrl+B, then D)"
    echo "Pictures:        http://$PIADDR:8000 (overhead)"
    echo "Stop it:         ./run-fleet-pi.sh freeroam-stop"
    echo "CHARGE THE ROBOTS STRAIGHT AFTER IT (they finish at 10 %)." ;;
  freeroam-run)     # (inside the tmux session freeroam)
    H="${1:-4}"; shift || true
    if [ "$DRY" = 1 ]; then
      python3 tools/fleet/freeroam.py run --dry --hours "$H" "$@" ${FREEROAM_DRY_ARGS:-}
    else
      PIN=""; if [ "$(nproc)" -ge 4 ] && command -v taskset >/dev/null; then PIN="taskset -c 2,3"; fi
      $PIN python3 tools/fleet/freeroam.py run --hours "$H" "$@" $IPS --stream 127.0.0.1:8000 --calib "$CALIB" $SIDE_ARG
    fi ;;
  freeroam-finish)  # (inside the tmux session freeroam, after the run): recorder stopped, logs + frames uploaded
    D="$(ls -td logs/*-freeroam 2>/dev/null | head -1)"
    [ "$DRY" = 1 ] && D="$(ls -td logs/*-freeroam-dryrun 2>/dev/null | head -1)"
    [ -f logs/freeroam-screen.txt ] && [ -n "$D" ] && mv logs/freeroam-screen.txt "$D/screen.txt"
    [ "$DRY" = 1 ] && { echo "(dry run: no recorder to stop, nothing uploaded; folder $D)"; exit 0; }
    OUT="$(cat /tmp/fleet_rec_out 2>/dev/null || echo "$HOME/recordings")"
    if rec_running; then tmux send-keys -t fleet:rec C-c; echo "Stopping the recorder ..."; sleep 6; fi
    [ -z "$D" ] && { echo "No free-roam folder found."; exit 1; }
    FR="$(python3 tools/fleet/diskguard.py where "$D")"
    # (7 Nov) program's-eye replay clips, in the background beside the upload (capped, never stops or delays it); off: MIRLT_REPLAY_CLIPS=0
    [ "${MIRLT_REPLAY_CLIPS:-1}" != 0 ] && { setsid nohup timeout 50000 nice -n 15 python3 tools/analysis/replay_clips.py auto "$D" --video "$FR" --upload > "$D-replay.log" 2>&1 < /dev/null & }
    python3 tools/pi/upload_session.py --recordings "$OUT" --session "$D" --tag "freeroam-$(basename "$D")" --unique-tag \
      --exclude frames --recording-dir "$FR" || echo "Upload failed: the files stay in $D and $FR (run ./run-fleet-pi.sh freeroam-finish again)" ;;
  freeroam-status)
    D=""; [ "$DRY" = 1 ] && D="--dry"
    python3 tools/fleet/freeroam.py status $D
    tmux has-session -t freeroam 2>/dev/null && { echo "--- its screen (last 12 lines) ---"; tmux capture-pane -p -t freeroam | grep -v '^$' | tail -12; } || echo "(no free roam running)" ;;
  freeroam-stop)
    if tmux has-session -t freeroam 2>/dev/null; then
      tmux send-keys -t freeroam C-c; echo "Sent Ctrl+C to the free roam: every robot stops; it then makes its report and uploads (./run-fleet-pi.sh freeroam-status)."
    else echo "No free roam running."; fi ;;
  monday|practice)
    # (e08, A10) the 2 Nov Monday demo's machinery is switched off by default (kept, not deleted): MIRLT_MONDAY_DEMO=1
    if [ "${MIRLT_MONDAY_DEMO:-0}" != 1 ]; then
      echo "Monday's demo mission (2 Nov) is switched off (e08). To run it anyway: MIRLT_MONDAY_DEMO=1 ./run-fleet-pi.sh $WHAT $*"
      exit 1
    fi
    TABLE="61x122"; ROB="Left,Right"; EXTRA=""; TS="monday"; PROG=""
    if [ "$WHAT" = practice ]; then
      # one block of Monday's mission alone (session plan 2026-10-28): mow | circle | chicken | dance
      B="${1:-}"; PF="tools/fleet/examples/practice/$B.json"
      [ -n "$B" ] && [ -f "$PF" ] || { echo "Which block? ./run-fleet-pi.sh practice mow|circle|chicken|dance [61x122]"; exit 1; }
      shift; TS="practice"; PROG="--program $PF"
      ROB="$(python3 -c "import json,sys; print(','.join(json.load(open(sys.argv[1]))['robots']))" "$PF")"
    fi
    for x in "$@"; do case "$x" in [0-9]*x[0-9]*) TABLE="$x" ;; *,*) ROB="$x" ;; farm) EXTRA="$EXTRA --farm --rings" ;; esac; done
    if [ "$DRY" = 1 ]; then
      python3 tools/fleet/mission_run.py --table "$TABLE" --robots "$ROB" $PROG --dry ${MONDAY_DRY_ARGS:-}
    else
      python3 tools/fleet/diskguard.py preflight || exit 1
      start_recorder
      OUT="$(cat /tmp/fleet_rec_out 2>/dev/null || echo "$HOME/recordings")"
      # cores: the recorder on 0-1, the camera bridge's workers on 1-2 (--farm), the bridge and run_plan on 3
      PIN=""; if [ "$(nproc)" -ge 4 ] && command -v taskset >/dev/null; then PIN="taskset -c 2,3"; fi
      # in tmux (session "monday"), so a dropped WiFi or a closed window does not stop the run half-way
      for s in monday practice doe ladder ladder2; do
        [ "$s" = "$TS" ] && continue
        close_if_ended $s
        tmux has-session -t $s 2>/dev/null && { echo "A run is going in tmux session $s: stop it first (./run-fleet-pi.sh stopall). Stopped."; exit 1; }
      done
      tmux kill-session -t "$TS" 2>/dev/null || true
      tmux new-session -s "$TS" "cd '$REPO' && $PIN python3 tools/fleet/mission_run.py --table '$TABLE' --robots '$ROB' $PROG --calib '$CALIB' ${SIDECAL:+--side-calib '$SIDECAL'} --upload --recordings '$OUT' $EXTRA; read -r -p 'Mission ended. Press Enter.'"
      echo "If the window closed: tmux attach -t $TS   (Ctrl+C in it stops every robot)"
    fi ;;
  mc)
    if python3 -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8095/api/run', timeout=2)" 2>/dev/null; then
      echo "Mission Control is already open: http://$PIADDR:8095 in the laptop's browser"
      tmux has-session -t mc 2>/dev/null && echo "It runs in tmux session mc. See it: tmux attach -t mc (leave: Ctrl+B, then D). Close it: ./run-fleet-pi.sh mc-stop"
      exit 0
    fi
    # (2026-11-03, risk review #5) Mission Control runs inside tmux session "mc", so it keeps running when the SSH window
    # closes or the laptop sleeps. `./run-fleet-pi.sh mc here` keeps the old way (in this window; Ctrl+C stops it).
    # Already inside tmux (the start page's Mission Control tab starts it in tmux mc): runs here.
    if [ -z "${TMUX:-}" ] && [ "${1:-}" != here ]; then
      tmux kill-session -t mc 2>/dev/null || true          # a closed one's window, waiting for Enter
      tmux new-session -d -s mc "cd '$REPO' && bash ./run-fleet-pi.sh mc here; read -r -p 'Mission Control stopped. Press Enter.'"
      echo "Mission Control is starting in tmux session mc (about 10 s) ..."
      for k in $(seq 1 30); do
        python3 -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8095/api/run', timeout=2)" 2>/dev/null && break
        tmux has-session -t mc 2>/dev/null || break; sleep 1
      done
      if python3 -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8095/api/run', timeout=2)" 2>/dev/null; then
        echo "Mission Control is open: http://$PIADDR:8095 in the laptop's browser"
      else
        echo "Mission Control did not answer yet. See its window: tmux attach -t mc (leave: Ctrl+B, then D)"
      fi
      echo "It keeps running if this window closes."
      echo "See its window:   tmux attach -t mc        (leave it running: Ctrl+B, then D)"
      echo "Close it:         ./run-fleet-pi.sh mc-stop"
      exit 0
    fi
    # (e12, Nick 10 Nov) the USB stick (ext4, label FLEET) holds the frames and video: said loudly when it is missing
    if ! bash tools/pi/mount_fleet.sh --check >/dev/null 2>&1; then
      echo "!! WARNING: the USB stick FLEET is NOT mounted: frames and video would go to the Pi's small card."
      echo "!! Plug it in, then type: tools/pi/mount_fleet.sh   (it asks for the Pi's password; docs/how-to/usb-stick.md)"
    fi
    # (e07) where am I: home, school or a new area, from the cameras plugged in and this computer (site_detect.py);
    # the recorder then takes that site's picture size. Mission Control checks again with the pictures (corner tiles,
    # the measured table) and says what it decided. A site chosen with `site <name> pin` always wins.
    DET="$(python3 tools/fleet/site_detect.py --decide 2>/dev/null)"
    if [ -n "$DET" ] && [ "$DET" != "${MIRLT_SITE:-}" ] && SITE_ENV2="$(MIRLT_SITE="$DET" python3 tools/fleet/sites.py env 2>/dev/null)"; then
      echo "This looks like the site $DET (its cameras / this computer): using it."
      eval "$SITE_ENV2"; IPS="$MIRLT_ROBOT_ARGS"
      CAPTURE="${MIRLT_CAPTURE:---width 640 --height 480}"; CAPFPS="${MIRLT_CAPTURE_FPS:-25}"
    fi
    # the recorder (its pictures for the robots' runs); in a subshell, so that an E02 session holding the cameras
    # leaves Mission Control open for planning and the simulator instead of stopping here
    if [ "$DRY" != 1 ]; then
      ( start_recorder ) || echo "(No recorder: Mission Control opens for planning and the simulator only; robot runs need the cameras.)"
    fi
    # the addresses the page may make changes from (its Host check): Tailscale, the Pi's name on the WiFi, its WiFi number
    HOSTS="--allow-host 100.122.222.75 --allow-host $(hostname).local"     # (home's Tailscale address kept: home's Pi)
    for ip in $(hostname -I 2>/dev/null); do case "$ip" in *.*.*.*) HOSTS="$HOSTS --allow-host $ip" ;; esac; done
    echo "Mission Control: http://$PIADDR:8095 in the laptop's browser (Ctrl+C here stops it)"
    [ -n "${MIRLT_PI_ADDR:-}" ] && HOSTS="$HOSTS --allow-host $MIRLT_PI_ADDR"
    case "$PIADDR" in *.*.*.*) HOSTS="$HOSTS --allow-host $PIADDR" ;; esac
    for ts in $(tailscale ip -4 2>/dev/null); do HOSTS="$HOSTS --allow-host $ts"; done
    # (2026-11-04) the camera's focus again (autofocus off, focus_absolute 0: both reset at every unplug or restart)
    [ "$DRY" != 1 ] && { python3 tools/pi/camera_focus.py || true; }
    python3 tools/fleet/mission_control.py --site auto --listen 0.0.0.0 $HOSTS ;;
  site)            # (2026-11-02) which site: ./run-fleet-pi.sh site (shows it), ./run-fleet-pi.sh site school-conference
    if [ "${1:-}" = auto ]; then python3 -c "import sys; sys.path.insert(0, 'tools/fleet'); import site_detect as SD; SD.set_pin(None)"; echo "Site: found by itself from now on (no site pinned)."
    elif [ "${2:-}" = pin ]; then python3 tools/fleet/sites.py use "$1" && python3 -c "import sys; sys.path.insert(0, 'tools/fleet'); import site_detect as SD; SD.set_pin('$1')" && echo "Pinned: $1 is always used (./run-fleet-pi.sh site auto undoes it)."
    elif [ -n "${1:-}" ]; then python3 tools/fleet/sites.py use "$1"; else python3 tools/fleet/sites.py; python3 tools/fleet/sites.py list; fi ;;
  jay-bundle)      # (2026-11-03) one run's raw files for Jay-Sun (camera track, raw odometry, logs; nothing RFID): a zip in the run's folder
    python3 tools/fleet/run_exports.py "${1:-latest}" --bundle ;;
  camcheck)        # (2026-11-02) the camera check-out in the terminal (the recorder must be running: ./run-fleet-pi.sh rec)
    python3 tools/fleet/camera_checkout.py ;;
  visit)           # (e11, Nick 9 Nov) Visit points in your order: ./run-fleet-pi.sh visit P01,P05,P11,P05 [keep] [dry]
    #                 one robot takes the whole list; keep = also points the camera cannot read; dry = practice (simulator)
    ORD="${1:-}"; [ -n "$ORD" ] || { echo "Which points? ./run-fleet-pi.sh visit P01,P05,P11   (add keep to visit points the camera cannot read; dry to practise)"; exit 1; }
    shift; VX=""; VDRY=0
    for x in "$@"; do case "$x" in keep) VX="$VX --visit-keep-unreadable" ;; dry) VDRY=1 ;; esac; done
    if [ "$VDRY" = 1 ]; then
      python3 tools/fleet/mission_run.py --tracking tags --robots auto --split auto --sim-robots 1 --motion stopturn --visit-order "$ORD" $VX --practice-real --feel-table skip
    else
      python3 tools/fleet/diskguard.py preflight || exit 1
      start_recorder
      for s in calib freeroam doe; do close_if_ended $s; done    # (e13) an ended run's window, not a running run
      tmux kill-session -t visit 2>/dev/null || true
      tmux new-session -s visit "cd '$REPO' && MIRLT_ATTENDED=1 python3 tools/fleet/mission_run.py --tracking tags --robots auto --split auto --motion stopturn --visit-order '$ORD' $VX --feel-table skip; read -r -p 'Visit ended. Press Enter.'"
      echo "If the window closed: tmux attach -t visit   (Ctrl+C in it stops every robot)"
    fi ;;
  mc-stop)
    if tmux has-session -t mc 2>/dev/null; then tmux send-keys -t mc C-c; sleep 3; tmux send-keys -t mc Enter 2>/dev/null; echo "Mission Control is closing (a run of its own stops first)."
    else echo "Mission Control is not running in tmux session mc: if it runs in a terminal window, Ctrl+C there."; fi ;;
  space)           # free space on the card and the USB drive, the biggest folders (nothing changed)
    python3 tools/fleet/diskguard.py space ;;
  clean-uploaded)  # only what GitHub has (quick check; --thorough: exact sizes); a list first, deletes only with --yes
    python3 tools/pi/clean_uploaded.py "$@" ;;
  uploads)         # (2026-11-05) the last 10 sessions, uploaded or not (quick check; nothing deleted)
    python3 tools/pi/clean_uploaded.py --status "$@" ;;
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
    echo "Recorder running. Its picture: http://$PIADDR:8000 in the laptop's browser" ;;
  check)
    python3 tools/fleet/robot_cmd.py $IPS --wait-s 3 status ;;
  stopall)         # everything that drives the robots stops, and `stop` straight to each robot (works mid-run): the
                   # site's robots, a broadcast `stop` and every robot heard on the WiFi (2026-11-03; risk review #2).
                   # `stopall listed`: the old way, only the listed addresses (home's three when the site has none)
    if [ "${1:-}" = listed ]; then python3 tools/fleet/stop_all.py --only-listed $IPS
    else python3 tools/fleet/stop_all.py $IPS; fi ;;
  calib)           # (2026-11-04) the calibration run (zones 1-2), in tmux session calib; `calib dry`: the simulator
    if [ "${1:-}" = dry ]; then python3 tools/fleet/calib_run.py --dry "${@:2}"
    else
      close_if_ended calib          # (e13) its run ended, the window waiting at "Press Enter": closed, a new run starts
      tmux has-session -t calib 2>/dev/null && { echo "A calibration run is already running: tmux attach -t calib (leave: Ctrl+B, then D)"; exit 0; }
      # (e08, A10, Nick 6 Nov) the robots sleep as soon as the run ends (as from the Start screen); `calib awake`
      # keeps the old way (awake; Mission Control's idle sleeper puts them to sleep after 30 minutes with no run)
      SLP="--sleep-at-end"; ARGS=""
      for x in "$@"; do case "$x" in awake) SLP="" ;; *) ARGS="$ARGS $x" ;; esac; done
      tmux new-session -d -s calib "cd '$REPO' && python3 tools/fleet/calib_run.py $IPS $SLP $ARGS; read -r -p 'Calibration run ended. Press Enter.'"
      echo "Calibration run started in tmux session calib. See it: tmux attach -t calib (leave: Ctrl+B, then D)"
    fi ;;
  calib-upload)    # (2026-11-04) the newest calibration run's folder (or the one given) as a release of its own
    F="${1:-$(ls -td "${MIRLT_MC_DIR:-logs/mission-control}"/calib/*/ 2>/dev/null | head -1)}"; F="${F%/}"
    [ -z "$F" ] && { echo "No calibration run folder found."; exit 1; }
    echo "Uploading $F ..."
    python3 tools/pi/upload_session.py --recordings "$HOME/recordings" --session "$F" --logs-only \
      --tag "calib-$(basename "$F")" --unique-tag ;;
  selfcheck)       # (2026-11-03) the new Pi's setup, tested: PASS / FAIL lines (tools/pi/selfcheck.py)
    python3 tools/pi/selfcheck.py "$@" ;;
  address)         # (2026-11-03) the Pi's addresses: Tailscale, WiFi, name, and the one the screens show
    python3 tools/fleet/pi_addr.py --all ;;
  settings)        # the settings read-back on the bench: send each robot its driving corrections, read them back
    R="${1:-all}"
    python3 tools/fleet/robot_cmd.py $IPS --to "$R" --wait-s 1.5 "set gyro_scale 1.075" --then "set wheel_scale 0.979" \
      --then "get gyro_scale" --then "get wheel_scale" ;;
  dance)           # the dance alone, where the robot stands, for about 8 s (firmware 2026-10-28a `run dance`), then stop
    R="${1:-Left}"
    python3 tools/fleet/robot_cmd.py $IPS --to "$R" --wait-s 8 "run dance" ;;
  turn)
    R="${1:-Center}"
    python3 tools/fleet/robot_cmd.py $IPS --to "$R" --wait-s 6 "turn 90" ;;
  calibrate)       # (2026-10-29) through the ladder: the camera's room check and roll caps; kept when done today
    case "${1:-}" in
      full) ladder calibrate --full ;;
      blind) python3 tools/fleet/robot_cmd.py $IPS --wait-s 75 calibrate ;;
      *) ladder calibrate ;;
    esac ;;
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
    sed -n '2,89p' "$0"; exit 1 ;;
esac
