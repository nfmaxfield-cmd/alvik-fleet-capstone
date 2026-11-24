- Every item of D1 to D6, N1 to N8 and the audit's findings and bugs has a change, a switch for the old way where
  behaviour changed, and a test or a recorded run, except the few listed under "Not done" below.
- In the simulator, on a stand-in built from the 23 Nov measurements, the build reached every point in all 17
  acceptance runs (320 of 320) where the code before reached 261 of 320 and said "done" for 7 runs that had missed
  points. The clearest gains are in the runs with a fault in them.
- *My own conclusions, not the team's:* (1) the main cause of 23 Nov's long waits was the planner rebuilding its grid
  on every re-plan, not the Pi's camera load; (2) the 18 Nov plate contact was not a heading fault; (3) the stand-in
  is now too kind to tell 4 + 4 runs apart. Each needs the next school session to confirm.

**Not done, and why**

- Audit finding 12's "a broken camera picture on `/classic` in practice before any run": practice now finds camera
  models from earlier runs, but I did not re-check that page state with no run on disk.
- Audit finding 12's "ended: timeout" wording against "never ends by itself": the time limit now follows the plan
  (Q-d), the wording of a run that does hit it was not reworked.
- The thrash was reproduced through the stand-in (the 20 + 20 before-run that ended at 8 of 40), not by replaying
  run 5's own logs through the simulator.
- Page behaviour is tested by Playwright scripts in the results folder, not inside the pytest suites.
- No firmware change was needed, so none was made.
