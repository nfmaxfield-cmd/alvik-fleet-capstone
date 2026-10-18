"""Hook for camera obstacle reports (new still objects on the floor).

Not built yet: a reliable finder needs a background picture of the empty
floor, a rule for what counts as "new and still" (not a robot standing still,
not a hand, not a shadow) and tests against hands and parked robots, because
every report makes run_plan.py stop the robots in its way and plan again
(a false one costs a re-plan). Until then track_live.py runs NoObstacles.

To add one: a class with update(t_pi, gray_picture, tracker) returning a list of
messages in run_plan.py's obstacle format (docs/data-formats.md "Position
source"), e.g.
    {"type": "obstacle", "id": "cam-1", "shape": "polygon",
     "points": [[x, y], ...], "source": "camera", "t_pi": t_pi}
each id reported once (run_plan ignores repeats). tracker.tracks gives the
robots' lights (to leave them out) and tracker.fmap.to_floor(u, v) turns pixels
into floor cm (for a point on the floor, use FloorMap with led_height_cm=0).
Register it in make() below and pick it with track_live.py --obstacles NAME.
"""


class NoObstacles:
    def update(self, t_pi, img, tracker):
        return []


FINDERS = {"none": NoObstacles}


def make(name):
    if name in (None, "", "none"):
        return None
    if name not in FINDERS:
        raise SystemExit("--obstacles must be one of: %s" % ", ".join(FINDERS))
    return FINDERS[name]()
