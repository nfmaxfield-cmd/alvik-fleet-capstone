# Open questions for Nick

Kept up to date by Claude (Nick, 2026-10-27 07:45: "Keep track of unanswered questions and I'll return to them later").
Answered ones move to the bottom with the answer and date.

## Open

### School demo (Monday)
1. What size is the school table? Monday's mission is fitted to an assumed 152 x 76 cm.
2. Are the build's routine changes OK: start on the Show's line-up spots (Center at the left end), Show about 54 s,
   Roam and Follow about 46 s each with Follow last? Or keep full 60 s blocks and let Left and Right finish after
   Center?
3. Chicken runs are short (10-24 cm) while Center mows. Keep that, or do Chicken before the mowing starts?
4. Should Mission Control become a tab on the live page now (it is a separate page, port 8095)?
5. The school WiFi name and password (they go only in your own `secrets.h`, never on GitHub). Firmware 2026-10-27a
   waits on it.
6. Connect the brick watch to the real camera and test it with your hands on the home board?
7. A time for the 45-minute board test (Phase 0: the planner on real robots). Plan and steps ready:
   [session plan 2026-10-27](session-plans/2026-10-27.md) (about 60 attended minutes with the bench checks).
14. Phase 0 gate: "within about 3 cm" read as each robot's 95th percentile from its lines (one camera jump does not fail
    a run). Or did you mean the maximum?
15. If Center's turns stay 15-20 deg off after calibrating, should Monday count on two robots? (Dry run: a robot like
    that cannot hold its lines even with camera fixes; the new camera guard stops it.)
16. If `check` shows 2026-10-26a or 27a on a robot: did you flash it last night, and from which commit?
17. Fix 3: after a barrier find the camera's edge watch is off for that robot for up to one route (90 s); its own
    border and edge sensors stay on. Acceptable, or limit it to a few seconds?
18. Should the inward rescue also apply to the other back-out ("try again on its own" step)? Same weakness.
19. A 2-minute who's-who check on the board today: drive one robot to stop about 13 cm from a parked one and watch
    for "who is who is checked" then "it is Left"?
20. Should the new DOE notes go into docs/doe-backlog.md too?

### Robots and sessions
8. What stands just beyond the home board's x = 0 edge at about y = 30-60? (Your 07:45 "yes" to this one was read as
   "yes, mark that side as partly blocked"; say if you meant something else.)
9. Run 4 of last night (Left, wrong camera identity): leave it out of the results, or use the video estimate (about
   33 cm, by eye)?
10. What cooling does the Pi have (you said yes to a fan or heatsink: which, and since when)? The heat tracker needs
    it in words.

### Analysis follow-ups
11. Should a robot's old sensor flag clear after N sessions with no new evidence (N = 1, 2 or 3 tested by replay), or
    show its date on the live page?
12. Is a speed-sweep test (4 / 7 / 10 / 13 cm/s) worth board time? It separates camera timing from wheel slip.
13. Should creep steps change? 12 of 23 stalled (faster creep, or end on the front sensor instead of the wheel count).

## Answered
- 2026-10-27 07:45: build fixes 1-4 from last night (yes; who's who may stop all robots to confirm); Nick saw Center
  drive off (yes, E1's 00:27:28 sidestep); Center and Left were still at the x = 122 edge (yes: bench `turn 90` check
  for Center first); the Pi has cooling (yes; which kind still open, question 10).
- 2026-10-27 07:08: standing still to re-plan is acceptable ("much rather ... than collide or go over the edge");
  the "plan only the next 30-60 s" re-plan work is dropped for now.
- 2026-10-27 00:08: re-plans send the first safe plan at once; Chicken 15 / 8 cm; nothing locked in the operator
  view; scope all of it, engine first.
- 2026-10-26 23:15: no music on Monday.
