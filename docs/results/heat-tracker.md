# Heat tracker: the Pi's temperature against the camera's speed

One row per session that logged cpu.csv (from 2026-10-26). Made by `tools/analysis/heat_tracker.py <session folder>`;
each session's chart is in `heat/`. States from the Pi's throttle flags at that moment: **throttling** = clock capped
or throttled now; **soft limit** = only the soft temperature limit now; **none** = no flag. Pictures a second = the
camera pictures the runner handled (perf.csv, of about 25). rho = Spearman rank correlation of temperature with
pictures a second (-1 to +1; near 0 = no link, or temperature barely varied).

| Session | Cooling | Temp median / max (C) | Min at 80 C+ | Time: none / soft limit / throttling | Pictures a second: none / soft / throttling (median) | Picture age 95% (s) | rho temp vs pictures |
|---|---|---|---|---|---|---|---|
| e02-20261026-095541 | none (bare Pi) | 84.2 / 86.2 | 68 | 4 / 80 / 17 % | 24.9 / 17.9 / 17.6 | 0.24 | -0.24 |

## Readings (newest first)
- **e02-20261026-095541 (bare Pi, no fan):** the Pi reached 83 C within 5 minutes and stayed at 83-86 C for the rest
  of the 70 minutes, under the soft temperature limit 80% of the time and actually throttling 17%. The camera handled
  a median 24.9 pictures a second in the few minutes with no flag, against 17.6-17.9 with one. But the temperature
  barely varied after minute 5, so this session cannot separate heat from workload (rho -0.24): the big steps in
  picture rate line up with what the runner was doing (e.g. 25 a second at minutes 42-52, while it waited on a
  question), not with temperature. The comparison that settles it: the same session plan with a fan or heatsink case
  on the Pi (the "Cooling" column).
  ![Pi temperature and camera speed, 2026-10-26](heat/e02-20261026-095541.png)
  The Pi's temperature (orange, red dots where it was throttling) and the pictures the camera handled each second
  (green), over the session; data: the session's cpu.csv and perf.csv.
