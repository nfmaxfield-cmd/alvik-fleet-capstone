# E1b: Low battery (part of T1/T2)

Status: planned    Week: 5 (Oct 18)    Run dates:

## Question
Do the robot's own sensors get less accurate when its battery is low?
Right was at 17% on Oct 17, which gives a low-battery set without waiting.

## Hypotheses
- (a) At low battery, wheel-only end error is larger than at full battery on
  the same robot and routes (slower, weaker motors slip more in turns).
  H0: no difference.
- (b) Gyro bias and noise at rest do not change with battery (the gyro has
  its own regulated supply). H0: no difference.

## Variables
| Kind | Variable | Levels or units |
| --- | --- | --- |
| Independent | battery | low (start of this block, Right below 25%) vs full (Right's E1 runs after charging to 100%) |
| Independent | route | same three as E1, 4 each |
| Dependent | end position and heading error, wheels only and wheels + gyro | cm, deg, tape |
| Dependent | gyro bias and noise in still periods; wheel speed vs command | deg/s; RAW |
| Logged | battery every run (RUNINFO, RUNEND) and 50 times a second (RAW) | robot log |
| Held constant | robot (Right), floor, routes, firmware 2026-10-18a | |

## Procedure
1. Tonight: switch Right **off** and do **not** charge it. Charge Left and Center.
2. After T0, before E1: run this sheet first, with Right only. Mark the
   start spots and start the camera as in E1's procedure (steps 1 and 2), then
   on the laptop, in the repo folder:
   `python tools/preflight.py --robots Right --pi st1maxfini.local --low-battery-ok Right`
   `python tools/run_sheet.py docs/experiments/E01b-low-battery --pi st1maxfini.local --inches`
3. Stop the block if Right drops below 8% or switches off mid-run; mark that
   run excluded with the reason. Fewer runs is fine; note how many.
4. Charge Right to 100%, then run its rows of the E1 sheet as normal. Those
   are the full-battery half of the comparison.

## Analysis
Paired: each low-battery run is matched with Right's full-battery E1 run of
the same route and the same place in that route's order on the sheet (1st low
square with 1st full square, and so on). Paired t-test on the differences
(low minus full) of end error (wheels only, then wheels + gyro from the
fusion replay), and of gyro bias and noise at rest. Pairing takes the route
out of the comparison, so fewer runs are needed than for two separate groups.
`python tools/accuracy_checks.py docs/experiments/E01-baseline-dataset docs/experiments/E01b-low-battery`
does this (checks 5 and 13). Battery is also a column on every E1 run, so
the full E1 set gets a battery-vs-error trend as a check (check 10).
Caveat: the low and full halves are different power-ons, so a change in gyro
bias between them can be the new power-on rather than the battery.

## Deviations
