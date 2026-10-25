Diagnosis scripts for real session e02-20261025-151004 (build report docs/build-reports/e02-oct25-session-1510.md).
Logs only. Inputs are hard-coded to where they were downloaded (GitHub release e02-session-20261025-151004):
  /tmp/claude-0/s1025/logs/e02-20261025-151004/   (logs zip)
  /tmp/claude-0/s1025/rec/20261025-150957_E02/     (recorder small zip)
  /tmp/claude-0/s1025/vid/cam0_video.avi.part-aa   (overhead video; frames.py pulls single frames into /tmp/claude-0/s1025/fr)
trk.py        track.csv -> pixels -> the stored (roam1) floor; `python3 trk.py 15:14:15 15:14:23 Right 0.2`
refit.py      explore's fit, contacts, rectangle and rect_check re-run from e02_state.json (reproduces the Pi's numbers)
cmp.py        explore's fitted floor against the stored calibration's floor
calcheck.py / prof.py   the stored outline against the plywood's edges in a video frame
edgestops.py  robot centre at every edge stop against the stored lines
figs.py       the two figures in the report
