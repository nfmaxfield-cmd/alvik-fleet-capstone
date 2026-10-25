"""The front sensor's offset and tolerance, learned every session (Nick, 2026-10-25: "devise a way for that to be set
automatically with each new session, to catch any unknown changes").

The residual is the front sensor's reading minus the map's distance to the board ahead (square on, a measured
heading, within FRONT_CHECK_RANGE_CM; e02.py front_check). From the 2026-10-23 and 2026-10-24 sessions (115
readings): median -1.1 cm, spread (MAD x 1.48) 5.5 cm; per robot Left -3.4 / Center -0.9 / Right +1.2 cm. That
spread includes the camera's own error (the map's position is the camera's), not only the sensor's.

How it works, per robot:
  - before this session has SESSION_MIN readings: the offset and spread from earlier sessions (logs/front_learned.json,
    the last PRIOR_KEEP readings of each robot), or the fleet's defaults (DEFAULT_OFFSET_CM / DEFAULT_SD_CM) for a
    robot never seen;
  - from SESSION_MIN readings on: this session's own (so a sensor knocked, swapped or re-mounted is caught at once,
    never judged against an old value);
  - the tolerance is TOL_SD x the spread, at least MIN_TOL_CM;
  - the moment this session's offset is first known, it is compared with the earlier one: a shift beyond
    max(SHIFT_MIN_CM, SHIFT_SE x its standard error) is reported ("changed").
At the end (save) this session's readings join the robot's history.

Tests: tests/test_e02_front.py."""
import json
import math
import os
import statistics

DEFAULT_OFFSET_CM = -1.1    # measured: 115 readings, 2026-10-23 and 2026-10-24 sessions (median of reading - map)
DEFAULT_SD_CM = 5.5         # measured: their spread (MAD x 1.48)
TOL_SD = 3.0                # chosen: 3 sd keeps 99.7% of normal readings inside (DOE note: the backlog's swap test
                            # sets the flag limits; this one only decides "disagrees with the map")
MIN_TOL_CM = 3.0
SESSION_MIN = 8             # chosen: readings before a session's own value replaces the earlier one (the median of 8
                            # has a standard error of about 0.4 sd; DOE note: a replay of sessions at 4 / 8 / 16
                            # would show how soon a shifted sensor is caught against false "changed" reports)
SHIFT_MIN_CM = 3.0          # chosen: a change smaller than this is not reported
SHIFT_SE = 3.0              # ... nor one within 3 standard errors of the earlier value
PRIOR_KEEP = 200            # readings kept per robot between sessions


def _med_sd(v):
    med = statistics.median(v)
    sd = statistics.median([abs(x - med) for x in v]) * 1.4826
    return med, sd


class FrontModel:
    def __init__(self, path=None):
        self.path = path
        self.prior = {}
        self.session = {}
        self.reported = set()
        if path and os.path.exists(path):
            try:
                with open(path) as f:
                    self.prior = {k: [float(x) for x in v] for k, v in (json.load(f).get("robots") or {}).items()}
            except (OSError, ValueError):
                self.prior = {}

    def _prior(self, robot):
        v = self.prior.get(robot) or []
        if len(v) >= SESSION_MIN:
            med, sd = _med_sd(v)
            return med, max(sd, 1.0), len(v), "earlier sessions"
        return DEFAULT_OFFSET_CM, DEFAULT_SD_CM, 0, "fleet default"

    def limits(self, robot):
        """(offset cm, tolerance cm, where it came from)."""
        v = self.session.get(robot) or []
        if len(v) >= SESSION_MIN:
            med, sd = _med_sd(v)
            return med, max(MIN_TOL_CM, TOL_SD * max(sd, 1.0)), "this session (%d readings)" % len(v)
        med, sd, n, src = self._prior(robot)
        return med, max(MIN_TOL_CM, TOL_SD * sd), src

    def add(self, robot, residual):
        """One reading - map. Returns words when this session first shows the sensor reading differently from
        before (a changed sensor), else ''."""
        v = self.session.setdefault(robot, [])
        v.append(float(residual))
        if len(v) != SESSION_MIN or robot in self.reported:
            return ""
        self.reported.add(robot)
        med_s, sd_s = _med_sd(v)
        med_p, sd_p, n_p, src = self._prior(robot)
        se = math.hypot(1.2533 * sd_s / math.sqrt(len(v)), 1.2533 * sd_p / math.sqrt(n_p) if n_p else sd_p)
        shift = med_s - med_p
        if abs(shift) > max(SHIFT_MIN_CM, SHIFT_SE * se):
            return ("%s's front sensor reads %+.1f cm differently from %s (%.1f vs %.1f cm, %d readings): this "
                    "session's value is used" % (robot, shift, src, med_s, med_p, len(v)))
        return ""

    def save(self):
        if not self.path:
            return
        out = {k: list(v) for k, v in self.prior.items()}
        for k, v in self.session.items():
            out[k] = (out.get(k, []) + list(v))[-PRIOR_KEEP:]
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(dict(robots={k: [round(x, 2) for x in v] for k, v in out.items()}), f)
        os.replace(tmp, self.path)
