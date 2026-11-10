#!/usr/bin/env bash
# school_pull.sh (2026-11-10e13, build e13): on the HOME Pi, copy new school run folders from the school Pi
# (RFIDTHESIS, shared to Nick's home Tailscale account by Jay-Sun) over ssh/rsync, then send each finished one to GitHub
# from the home Pi (as decided 9 Nov: no GitHub key on the school Pi).
#
#   NEVER deletes anything at either end: rsync without --delete; a home file replaced by a newer school version (a run
#   still being written) is kept under ~/school-runs/.replaced/<time>/; the upload with --keep-local (no auto-clean).
#   RFID: files with "rfid" in their name are never copied (raw RFID data stays private: CLAUDE.md).
#   Pictures (frames, recordings) are copied only when the school Pi's USB stick is mounted; they are not uploaded
#   unless SCHOOL_UPLOAD_PICTURES=1 (the logs are).
#   One copy at a time (flock): an hourly run that finds the last one still going leaves at once.
#
#     tools/pi/school_pull.sh              copy, then upload the finished runs not uploaded yet
#     tools/pi/school_pull.sh --test       only check the ssh login and list what would be copied (nothing copied)
#     tools/pi/school_pull.sh --no-upload  copy only
#
# Settings (environment, all optional): SCHOOL_PI (default nick@100.72.183.28), SCHOOL_KEY (~/.ssh/school_pi),
# SCHOOL_REPO (the repo folder on the school Pi, default alvik-fleet-capstone in its home), SCHOOL_STICK
# (/media/nick/FLEET), SCHOOL_DEST (~/school-runs), SCHOOL_QUIET_MIN (15: a run folder untouched this long is
# finished; CHOSEN, no DOE: every run writes its state at least once a minute). Tests only: SCHOOL_SSH (the ssh
# command), SCHOOL_UPLOAD (the upload command).
# The one-time setup: docs/how-to/school-runs-to-home.md.
set -u
VERSION="2026-11-10e13"
SCHOOL_PI="${SCHOOL_PI:-nick@100.72.183.28}"
SCHOOL_KEY="${SCHOOL_KEY:-$HOME/.ssh/school_pi}"
SCHOOL_REPO="${SCHOOL_REPO:-alvik-fleet-capstone}"
SCHOOL_STICK="${SCHOOL_STICK:-/media/nick/FLEET}"
DEST="${SCHOOL_DEST:-$HOME/school-runs}"
QUIET_MIN="${SCHOOL_QUIET_MIN:-15}"
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
STATE="${MIRLT_STATE_DIR:-$HOME/.local/state/mirlt}"
UPLOADED="$STATE/school_uploaded.txt"
LOG="$STATE/school_pull.log"
MODE="${1:-}"
mkdir -p "$DEST/logs" "$DEST/stick" "$STATE"
touch "$UPLOADED"

say() { echo "$(date '+%Y-%m-%d %H:%M:%S') $*" | tee -a "$LOG"; }

exec 9>"$STATE/school_pull.lock"
if ! flock -n 9; then
  say "another school_pull is still running: nothing done this time"
  exit 0
fi

SSH=(${SCHOOL_SSH:-ssh} -i "$SCHOOL_KEY" -o BatchMode=yes -o ConnectTimeout=20 -o StrictHostKeyChecking=accept-new)
if ! "${SSH[@]}" "$SCHOOL_PI" true 2>>"$LOG"; then
  say "!! cannot log in to the school Pi ($SCHOOL_PI): is Tailscale on here, the share accepted, the key copied? (docs/how-to/school-runs-to-home.md)"
  exit 2
fi
say "school_pull $VERSION: logged in to $SCHOOL_PI"

STAMP="$(date +%Y%m%d-%H%M%S)"
# a file that changed at school (a run still being written) is copied again; the home copy it replaces is KEPT under
# $DEST/.replaced/<time>/ (nothing is ever deleted)
RS=(rsync -a --partial --backup --backup-dir="$DEST/.replaced/$STAMP" --exclude '*rfid*' --exclude '*RFID*' --exclude '.upload-tmp' -e "${SSH[*]}")
if [ "$MODE" = "--test" ]; then
  say "TEST: what would be copied (nothing is copied):"
  "${RS[@]}" --dry-run --itemize-changes "$SCHOOL_PI:$SCHOOL_REPO/logs/" "$DEST/logs/" | tail -n 20
  if "${SSH[@]}" "$SCHOOL_PI" "test -d '$SCHOOL_STICK'"; then
    say "TEST: the school Pi's USB stick is mounted ($SCHOOL_STICK): its pictures would be copied too"
  else
    say "TEST: the school Pi's USB stick is not mounted: logs only"
  fi
  exit 0
fi

# 1. the logs: every run folder (command line logs/<date-time>-*, Mission Control's logs/mission-control-*/runs)
"${RS[@]}" "$SCHOOL_PI:$SCHOOL_REPO/logs/" "$DEST/logs/" 2>>"$LOG" && say "logs copied to $DEST/logs" \
  || say "!! the logs copy stopped part-way (it continues next time)"
# 2. the pictures, when the stick is mounted there
if "${SSH[@]}" "$SCHOOL_PI" "test -d '$SCHOOL_STICK'"; then
  for sub in fleet-frames recordings; do
    if "${SSH[@]}" "$SCHOOL_PI" "test -d '$SCHOOL_STICK/$sub'"; then
      "${RS[@]}" "$SCHOOL_PI:$SCHOOL_STICK/$sub/" "$DEST/stick/$sub/" 2>>"$LOG" && say "pictures copied: $sub" \
        || say "!! the pictures copy ($sub) stopped part-way (it continues next time)"
    fi
  done
else
  say "the school Pi's USB stick is not mounted: pictures not copied this time"
fi
[ "$MODE" = "--no-upload" ] && exit 0

# 3. each finished run folder not uploaded yet: to GitHub from here (the home Pi's token), the home copy kept
n=0
while IFS= read -r d; do
  [ -d "$d" ] || continue
  grep -qxF "$d" "$UPLOADED" && continue
  if [ -n "$(find "$d" -type f -newermt "-$QUIET_MIN minutes" -print -quit 2>/dev/null)" ]; then
    continue                                  # still being written (or copied): next time
  fi
  name="$(basename "$d")"
  extra=(--logs-only)
  [ "${SCHOOL_UPLOAD_PICTURES:-0}" = 1 ] && extra=()
  if ${SCHOOL_UPLOAD:-python3 "$REPO/tools/pi/upload_session.py"} --recordings "$DEST/stick/recordings" --session "$d" \
       --tag "school-$name" --unique-tag --keep-local "${extra[@]}" >>"$LOG" 2>&1; then
    echo "$d" >> "$UPLOADED"
    say "uploaded: $name (release school-$name)"
    n=$((n + 1))
  else
    say "!! upload failed: $name (tried again next time; the log above says why)"
  fi
done < <(find "$DEST/logs" -mindepth 1 -maxdepth 1 -type d -name '20*' 2>/dev/null; \
         find "$DEST/logs" -mindepth 3 -maxdepth 3 -type d -path '*/mission-control-*/runs/*' 2>/dev/null)
say "done: $n run folder(s) uploaded this time"
