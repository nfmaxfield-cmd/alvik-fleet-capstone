# Changes Jay-Sun made on the school Pi

Jay-Sun's own log of what he (and his AI) changed on the school Pi, in his words, as it stood on the Pi on 23 Nov 2026
(commits `55a5c53`, `1a5a58c`, `1fa268e`). Brought into the repo on 24 Nov 2026 (build e24), with a line under each
entry saying where it lives now. New entries go at the end.

**From 24 Nov on, a change that belongs to this one Pi goes in the Pi's own file, not in the repo's site file** (so the
Pi never differs from GitHub and `./run-fleet-pi.sh update` stays one command):

```
./run-fleet-pi.sh local                               # what is set on this Pi
./run-fleet-pi.sh local set cameras.side.use false    # camera 2 off on this Pi
./run-fleet-pi.sh local set capture.fps 15            # the capture rate by hand
./run-fleet-pi.sh local unset capture.fps             # back to the automatic rate
```

## 2026-11-18 (Jay-Sun): camera 2 turned on at the school site
- File: tools/fleet/sites/school-conference.json, cameras.side.use false -> true (one line).
- Why: second EMEET C950 added at the door end so roof-tag runs track zones 3-4 (Experiment C).
- Camera names set in ~/.config/mirlt/sites/school-conference/cameras.json by USB port (overhead EMEET 1.1, side EMEET 1.2.2; backup cameras.json.bak-1118). Side camera measured in Cameras & table (tagcal/side.json, 14:21).
- Tested: a short tour into zones 3-4 after restarting Mission Control (see below).
- *Now (e24):* in the repo: the school site file says `cameras.side.use true`.

## 2026-11-18 (2)
- tools/fleet/mission_control.html: Visit points card has a "Show camera: overhead | side (camera 2)" switch; picture, sticker marks and unreadable rings load from the chosen camera (was fixed to overhead). Falls back to overhead if side fails.
- ~/.config/mirlt/sites/school-conference/cameras.json (not in git): side camera moved to the blue USB port, port 1.2.2 -> 1.4.2 (backup cameras.json.bak-1118b).
- *Now (e24):* in the repo, on the old page as written and on the new page's plan view too. The fall-back now also works for a camera that has no model yet (before, the server answered without an error and the fall-back never ran).

## 2026-11-23
- tools/fleet/sites/school-conference.json: capture fps 15 -> 10 (two cameras decoding on core 0 at 100%, picture lag ~4 s, robots standing still waiting on 20+ s re-plans). Backup school-conference.json.bak-1123.
- Stopped leftover background site_fit refits by PID.
- Recalibrated Alvik 4 (AL-EB54): wheel_scale 0.9743, gyro_scale 1.0790, gyro_drift_dps 0.0037 (calibration was done at 15 fps, then set back to 10).
- *Now (e24):* the school starts at 10 in the repo, and the rate then follows the Pi's measured load after each real run (`python3 tools/pi/capture_rate.py show`). The 23 Nov runs showed no measured difference between 15 and 10 (docs/results/school-2026-11-23/report.md, section 2.6). The AL-EB54 values were not in force in any 23 Nov run (same report, section 2.9); why is in docs/build-reports/e24-nov24-school-and-usability.md.
