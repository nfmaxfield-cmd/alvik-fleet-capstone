# Mission Control and the school demo: build brief (2026-10-27 00:10)

Source: the Mission Control plan doc (https://claude.ai/artifact/4ZHA1JETwR1UtR76aZGmMF), the mockup
(https://claude.ai/artifact/ByAd3Gf8eEj5ARiDrVsHn7), and Nick's answers up to 2026-10-27 00:03. This file is the brief
the build works from.

## The demo (Monday 2026-11-02, a school)

- A new table with **no boards**, an overhead camera at an unknown, roughly-overhead spot, a bright room, the room's
  own network. The laptop runs everything. People catch falls; still: no collisions, ever.
- Routine: **Show** (choreographed, about 45 s, all three, **no music**) → split: **Center mows** (one full pass,
  lanes 12 cm apart, 10 cm in from every edge; right of way) while **Left and Right** do **Chicken** twice, **Follow
  the leader** 60 s, **Roam** 60 s → when Center finishes, all three **park in a row**.
- Nick wants to **drop a brick on the table mid-run** and have the robots react: the camera spots it, the robot that
  would meet it stops, the others keep driving, routes and the mowing lanes go round it.
- Not graded: the point is showing control in a new place.

## Nick's decisions

| Decision | Answer |
|---|---|
| Six-screen journey (Connect, Workspace, Warm-up, Mission, Run, Review) + a Details switch | As planned (mockup: Workspace, Mission, Run screens) |
| Spots | Labelled A, B, C...; the instruction list decides order and looping ("go to A · wait 5 s · go to C · repeat 3×") |
| Platform | Computer only (desktop browser); simple and clean, few options up front, depth behind "More"/Details |
| Audience | Nick now; designed to illustrate a handover to a plant operator |
| Music | **None** for Monday |
| Chicken | Turn away at **15 cm**, never under **8 cm** (8 = two half-widths + ~3 cm camera error, rounded up; DOE on the board) |
| Operator view (Details off) | **All settings locked**: operators pick missions, spots, Run/Pause/Stop only |
| Re-plans | **Send the first safe plan at once**; keep searching in the background and swap in a shorter plan only if found (and safe to swap) |
| Warm-up | Explore-style, user-confirmed: robots **creep cautiously into each corner and pause** (those frames confirm the corners), **feel each side**: open edge, barrier, or partly both; things bumped away from a side become **no-go zones** for Nick to confirm/delete; frames where the tracker lost a robot go to the check too. Nick's confirming/editing time: **90 s or less** (about 15 frames). Confirmed frames are kept as labelled data for the detector later (not trained on for Monday). |
| Board | General shape everywhere: the table's rectangle + each side's kind (open / barrier / partial) + no-go zones. Nothing may assume two boards. |
| Firmware | Virtual border from the rectangle and each side's kind (all four sides), shipped together with the second WiFi network (home + school) so Nick flashes once |

## Order of work (simulator and logs only; never real robots)

Phase 1, the engine (stop at a clean, committed point if time runs out):

1. Re-plans: first safe plan at once, better plan swapped in later (tools/fleet planner + run_plan). Measure as in
   docs/build-reports/e02-oct26-noboards-and-replan.md (20 re-plans, old vs new).
2. General workspace map: rectangle, per-side kind, named spots, no-go zones (tools/fleet/fleetmap.py and the
   runner's board model e02_plan.Board: `missing` is the start of this).
3. Firmware: border from the rectangle and per-side kinds; second WiFi network (SSID/password left as placeholders
   until Nick's school checklist answers arrive). Host-compile with stand-in headers as before; never claim it was
   compiled with the Arduino tools.
4. Warm-up: corner creeps with pauses, side feeling (edge / barrier / partial), no-go zones from bumps, the frame
   list for Nick's 90 s check, stored-vs-measured wheel and gyro check.
5. Brick drop: camera "what changed" (e02_bricks floor memory) → planner obstacle event → re-plan; hands ignored;
   the mower re-lanes round it.
6. Run modes on the planner: Show (dance composer figures converted to planner moves, beat times as targets),
   Lawnmower, Go to spots + instruction lists, Chicken, Follow the leader, Roam; Monday's routine as one mission.

Phase 2, the Mission Control tab in the live lab (6 screens, Details switch, preview with time slider, finish time,
closest approach), styled like the mockup (Inter, ground #F6F7F9, white panels, border #E4E7EC, ink #101828, muted
#5B6474, accent #2952CC, green #12805C; robots Left #2952CC, Center #C2410C, Right #7A3EC8).

## Standing rules for the build

- Screenshots of features and problems to docs/screenshots/<date>/ with a line each in docs/screenshots/index.md.
- Every chosen number gets a DOE note in the code.
- Reports: premise → data → conclusion → next, ending with two sweeps (new tests; possible inaccuracies). Lean.
- Time tracker row in docs/time-tracker.md. Commit and push to main as each piece lands.
