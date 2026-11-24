#!/bin/bash
# (e24) Rehearsal of `./run-fleet-pi.sh update` on a scratch clone rebuilt to mirror the school Pi as it stood on
# 23 Nov 2026 (pi-git.txt): Jay-Sun's local commits 55a5c53 (camera 2 on), 1a5a58c (his camera switch), the merge of
# origin/main at 294d937, 1fa268e (capture 15 -> 10), then an edited site file that was not committed and two loose
# .bak files. "GitHub" is a bare copy of THIS repo's HEAD.
#     update_rehearsal.sh WORK_DIR [uncommitted-edit]      uncommitted-edit: fps15 (default) | sideoff | other
# Prints the update's own output and then what the checks found. Nothing outside WORK_DIR is touched
# (HOME is WORK_DIR/home, backups go to WORK_DIR/home/mirlt-backups).
# Changelog: 2026-11-24 first version.
set -e
W="$(realpath -m "$1")"; EDIT="${2:-fps15}"
SRC="$(cd "$(dirname "$0")/../../.." && pwd)"
rm -rf "$W"; mkdir -p "$W/home"
git clone -q --bare "$SRC" "$W/github.git"
git clone -q "$W/github.git" "$W/pi"
cd "$W/pi"
git config user.name "Jay-Sun"; git config user.email "jay-sun@rfidthesis.local"
# the Pi's history: it stood at e23 as built (b202ea5) when Jay-Sun made his first commit on 18 Nov
git checkout -q -B main b202ea5
python3 - <<'PY'
import re
p = "tools/fleet/sites/school-conference.json"; s = open(p).read()
s = s.replace('"use": false,\n   "note": "switched off 5 Nov 2026', '"use": true,\n   "note": "switched off 5 Nov 2026', 1); open(p, "w").write(s)
open("docs/jay-sun-changes.md", "w").write("\n## 2026-11-18 (Jay-Sun): camera 2 turned on at the school site\n- File: tools/fleet/sites/school-conference.json, cameras.side.use false -> true (one line).\n")
PY
git add -A; git commit -qm "Jay-Sun: turn camera 2 (side) back on at the school site for two-camera tracking"
python3 - <<'PY'
p = "tools/fleet/mission_control.html"; s = open(p).read()
s = s.replace('vpPic = 0, vpStamp = Date.now();', 'vpPic = 0, vpStamp = Date.now(), vpCam = "overhead";', 1)
s = s.replace('VP = await api("/api/visit/points?cam=overhead"); } catch (e) { VP = null; }', 'VP = await api(`/api/visit/points?cam=${vpCam}`); } catch (e) { VP = null; if (vpCam !== "overhead") { vpCam = "overhead"; } }', 1)
open(p, "w").write(s)
open("docs/jay-sun-changes.md", "a").write("\n## 2026-11-18 (2)\n- tools/fleet/mission_control.html: Visit points card has a \"Show camera: overhead | side (camera 2)\" switch.\n")
PY
git add -A; git commit -qm "Jay-Sun: Visit points card can show the side camera (camera 2); log camera 2 port move"
git merge -q --no-edit 294d937 -m "Merge remote-tracking branch 'origin/main'"
python3 - <<'PY'
p = "tools/fleet/sites/school-conference.json"; s = open(p).read()
assert '"fps": 15' in s; s = s.replace('"fps": 15', '"fps": 10', 1); open(p, "w").write(s)
open("docs/jay-sun-changes.md", "a").write("\n## 2026-11-23\n- capture fps 15 -> 10.\n")
PY
git add -A; git commit -qm "Jay-Sun: school site capture 15 -> 10 fps; log Alvik 4 recalibration"
# not committed: an edit to the site file, and two loose backup files
python3 - "$EDIT" <<'PY'
import sys
p = "tools/fleet/sites/school-conference.json"; s = open(p).read()
e = sys.argv[1]
if e == "fps15": s = s.replace('"fps": 10', '"fps": 15', 1)
elif e == "sideoff": s = s.replace('"use": true,\n   "note": "switched off 5 Nov', '"use": false,\n   "note": "switched off 5 Nov', 1)
else: s = s.replace('"edge_in_cm":', '"edge_in_cm_was_edited_here": 1,\n "edge_in_cm":', 1) if '"edge_in_cm":' in s else s.replace('{', '{\n "edited_here": 1,', 1)
open(p, "w").write(s)
PY
cp tools/fleet/mission_control.html tools/fleet/mission_control.html.bak-1118b
cp tools/fleet/sites/school-conference.json tools/fleet/sites/school-conference.json.bak-1123
echo "=== the rebuilt Pi before the update ==="; git log --oneline | head -5; git status --short
BEFORE_HEAD=$(git rev-parse HEAD); sha() { sha256sum "$1" | cut -c1-16; }
S1=$(sha tools/fleet/sites/school-conference.json); B1=$(sha tools/fleet/mission_control.html.bak-1118b)
export HOME="$W/home" MIRLT_UPDATE_NO_MC=1 MIRLT_SITE=school-conference
echo; echo "=== getting the update command onto the Pi (the first update's two steps) ==="
git fetch -q origin
git show origin/main:tools/pi/update_pi.py > "$W/update_pi.py"
echo "=== python3 <the fetched update_pi.py>, typed in the Pi's code folder ==="
set +e
python3 "$W/update_pi.py"; RC=$?
set -e
echo; echo "=== checks after the update (exit $RC) ==="
echo "HEAD == GitHub's newest: $([ "$(git rev-parse HEAD)" = "$(git rev-parse origin/main)" ] && echo yes || echo NO)"
BR=$(git branch --list 'pi-backup-*' | tr -d ' *' | head -1)
echo "backup branch $BR holds the Pi's old HEAD: $([ "$(git rev-parse "$BR")" = "$BEFORE_HEAD" ] && echo yes || echo NO)"
echo "Jay-Sun's 3 commits on it: $(git log --oneline "$BR" | grep -c 'Jay-Sun:')"
BD=$(ls -d "$W"/home/mirlt-backups/* | head -1)
echo "the edited site file's copy is byte-identical in the backup folder: $([ "$(sha "$BD/changed-files/tools/fleet/sites/school-conference.json")" = "$S1" ] && echo yes || echo NO)"
echo "stash kept: $(git stash list | head -1)"
echo "loose .bak files still in place and unchanged: $([ "$(sha tools/fleet/mission_control.html.bak-1118b)" = "$B1" ] && [ -f tools/fleet/sites/school-conference.json.bak-1123 ] && echo yes || echo NO)"
echo "working tree clean of tracked changes: $([ -z "$(git status --porcelain | grep -v '^??')" ] && echo yes || echo NO)"
echo "this Pi's own choices: $(cat "$W/home/.config/mirlt/sites/school-conference/site_local.json" 2>/dev/null | tr -d '\n' || echo none)"
python3 tools/fleet/sites.py local | tail -1
echo "second update (already up to date):"; python3 tools/pi/update_pi.py | tail -4
