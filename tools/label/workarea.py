"""Work area of a camera: the part of the picture where objects are looked for.

A work area is a polygon of corner points (image pixels) plus a margin band around it
(for things reaching in, e.g. a hand), and optional ignore spots inside it (a reflection,
a blinking light): circles x, y, r. Guesses outside the area and its margin, or inside an
ignore spot, are dropped before tracking. No polygon means the whole picture.

The recognition studio keeps one per camera in project.json:
    cameras[<camera name, or "default">] = {work_area: [[x, y], ...] | null,
                                            margin_px: number | null (null = one object size),
                                            ignore_spots: [[x, y, r], ...]}

    area = Area(polygon, margin_px, ignore_spots, default_margin=robot_px)
    area.keeps(x, y)              -> True / False
    filter_guesses(dets, area)    -> (dets with the dropped guesses removed, number dropped)
"""
import math


def inside_polygon(x, y, poly):
    """Even-odd rule: is (x, y) inside the polygon [(x, y), ...]?"""
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y):
            xc = xi + (y - yi) * (xj - xi) / ((yj - yi) or 1e-12)
            if x < xc:
                inside = not inside
        j = i
    return inside


def distance_to_polygon(x, y, poly):
    """Distance (px) from (x, y) to the polygon's outline."""
    best = float("inf")
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
    return best


class Area:
    def __init__(self, polygon=None, margin_px=None, ignore_spots=(), default_margin=30.0):
        self.polygon = [(float(x), float(y)) for x, y in polygon] if polygon and len(polygon) >= 3 else None
        self.margin = float(default_margin if margin_px is None else margin_px)
        self.spots = [(float(x), float(y), float(r)) for x, y, r in (ignore_spots or [])]

    @classmethod
    def from_settings(cls, settings, default_margin=30.0):
        """settings: a project.json cameras[...] entry (or None: keep everything)."""
        s = settings or {}
        return cls(s.get("work_area"), s.get("margin_px"), s.get("ignore_spots") or [], default_margin)

    @property
    def active(self):
        return bool(self.polygon or self.spots)

    def keeps(self, x, y):
        for sx, sy, r in self.spots:
            if math.hypot(x - sx, y - sy) <= r:
                return False
        if self.polygon is None:
            return True
        return inside_polygon(x, y, self.polygon) or distance_to_polygon(x, y, self.polygon) <= self.margin


def filter_guesses(dets, area):
    """dets: per frame a list of guesses (dicts with x, y) or None (frame not looked at).
    Returns (new list, number of guesses dropped). The input is not changed."""
    if area is None or not area.active:
        return dets, 0
    out, dropped = [], 0
    for ds in dets:
        if ds is None:
            out.append(None)
            continue
        keep = [d for d in ds if area.keeps(float(d["x"]), float(d["y"]))]
        dropped += len(ds) - len(keep)
        out.append(keep)
    return out, dropped
