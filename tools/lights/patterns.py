"""Light patterns on top of each robot: which LEDs sit where, and which code
is which robot. Shared by the detector (detect.py), the simulator
(tools/sim/camera.py) and the firmware's lights.h (same numbers, by hand).

Two kinds (tools/blink/README.md "Light patterns" has the design note):

  l3     three bright LEDs in an L on a holder on the robot's top: a corner
         LED, a FRONT LED arm_cm ahead of it (the long arm points the way the
         robot faces) and a SIDE LED side_cm to its left or right. The holder
         sets the geometry: side arm length (one of side_cm levels) and side
         (L/R) -> 6 holder variants with the default 3 levels, each readable
         in ONE frame, heading included (the L is not mirror-symmetric).
         On top of that the firmware can switch the SIDE LED on and off in a
         4-slot cycle (slot_s each): sequence 1111 (steady) / 0111 / 0011 /
         0101 / 0001 -> x5 codes, read over one cycle or more (0.64 s).
         Code strings: "S<level><side>" for steady, "S1L/0011" with a
         sequence. Corner and front LED are always lit.
  strip  an 8-LED row (Arduino Modulino Pixels) along the robot's forward
         axis showing a static bit pattern: LED 0 (front) and 1 lit = start
         (a double, so front and back differ: heading), LED 2 and 6 dark,
         LED 7 lit = stop, LEDs 3-5 carry the code (no two neighbours lit, so
         they stay apart in the picture): codes 1..5.

Robot frame: forward = +x, left = +y, cm, origin at the turning centre
(the point the pose refers to)."""
import math
import re
from dataclasses import dataclass, field

import numpy as np

SEQS = ("1111", "0111", "0011", "0101", "0001")   # side-LED cycles (necklaces of 4 slots; 1111 = steady)

STRIP_CODES = {1: "11000001", 2: "11010001", 3: "11001001", 4: "11000101", 5: "11010101"}

DEFAULT_L3_NAMES = {"Left": "S0L", "Center": "S1L", "Right": "S2L"}
DEFAULT_STRIP_NAMES = {"Left": "1", "Center": "2", "Right": "3"}


@dataclass
class L3Family:
    arm_cm: float = 5.0                      # corner -> front LED (along the robot's forward axis)
    side_cm: tuple = (2.0, 3.0, 4.0)         # corner -> side LED, one level per holder variant
    corner_cm: tuple = (-3.5, 0.0)           # corner LED in the robot frame (forward, left), from the turning centre:
    #                                          the front LED (1.5, 0) stays 2.8 cm from the robot's own top LEDs
    #                                          (2.75, +-2.55), so the blink code still reads and they can't form an L
    height_cm: float = 6.0                   # LED tops above the floor (robot top 4 cm + holder)
    slot_s: float = 0.16                     # sequence slot (4 camera frames at 25 fps)
    kind: str = "l3"

    @property
    def n_slots(self):
        return 4

    def geoms(self):
        return ["S%d%s" % (i, s) for i in range(len(self.side_cm)) for s in "LR"]

    def codes(self):
        return [g + ("" if q == "1111" else "/" + q) for g in self.geoms() for q in SEQS]

    @staticmethod
    def parse(code):
        """'S1L' -> ('S1L', '1111'); 'S1L/0011' -> ('S1L', '0011')."""
        m = re.fullmatch(r"(S\d[LR])(?:/([01]{4}))?", code.strip())
        if not m:
            raise ValueError("not an l3 code: %r (S<level><L|R>[/<4 bits>])" % code)
        q = m.group(2) or "1111"
        if q not in SEQS:
            raise ValueError("sequence %s is not one of %s (others are rotations of these)" % (q, ", ".join(SEQS)))
        return m.group(1), q

    def side_of(self, geom):
        lvl, s = int(geom[1]), geom[2]
        return self.side_cm[lvl] * (1.0 if s == "L" else -1.0)

    def leds(self, geom):
        """(3, 2) robot-frame positions: corner, front, side."""
        cx, cy = self.corner_cm
        return np.array([[cx, cy], [cx + self.arm_cm, cy], [cx, cy + self.side_of(geom)]], float)

    def centre(self, geom):
        """The point reported as the pattern's position: the mean of the three LEDs (robot frame)."""
        return tuple(self.leds(geom).mean(axis=0))

    def lit(self, code, t):
        """(corner, front, side) on/off at t seconds after the pattern started."""
        _, q = self.parse(code)
        slot = int(math.floor(t / self.slot_s)) % 4
        return (True, True, q[slot] == "1")

    def capacity(self):
        return dict(single_frame=len(self.geoms()), with_sequence=len(self.codes()))


@dataclass
class StripFamily:
    pitch_cm: float = 0.45                   # LED to LED (Modulino Pixels: 8 x LC8822-2020 on a 41 mm board;
    #                                          the pitch is not in the datasheet: ASSUMED, measure it)
    n: int = 8
    centre_cm: tuple = (0.0, 0.0)            # strip centre in the robot frame
    height_cm: float = 5.0                   # LEDs above the floor (robot top + board on a spacer)
    kind: str = "strip"

    def codes(self):
        return [str(k) for k in STRIP_CODES]

    @staticmethod
    def parse(code):
        k = int(str(code).strip())
        if k not in STRIP_CODES:
            raise ValueError("strip code must be one of %s" % sorted(STRIP_CODES))
        return k

    def bits(self, code):
        return [int(b) for b in STRIP_CODES[self.parse(code)]]

    def leds(self, code=None):
        """(8, 2) robot-frame positions, LED 0 at the front."""
        cx, cy = self.centre_cm
        return np.array([[cx + (3.5 - i) * self.pitch_cm, cy] for i in range(self.n)], float)

    def centre(self, code=None):
        return tuple(self.centre_cm)

    def lit(self, code, t=0.0):
        return tuple(b == 1 for b in self.bits(code))

    def capacity(self):
        return dict(single_frame=len(STRIP_CODES), with_sequence=len(STRIP_CODES))


def family(kind, **kw):
    if kind == "l3":
        return L3Family(**kw)
    if kind == "strip":
        return StripFamily(**kw)
    raise ValueError("pattern kind must be l3 or strip, not %r" % kind)


def parse_names(s, fam):
    """'Left=S0L,Center=S1L/0011' -> {name: code} (checked against the family);
    empty -> the default table."""
    if not s:
        return dict(DEFAULT_L3_NAMES if fam.kind == "l3" else DEFAULT_STRIP_NAMES)
    out = {}
    for part in s.split(","):
        if not part.strip():
            continue
        n, _, c = part.partition("=")
        c = c.strip()
        fam.parse(c)
        out[n.strip()] = c
    codes = list(out.values())
    if len(set(codes)) != len(codes):
        raise ValueError("two names share a lights code: %s" % s)
    return out


def min_pixels(fam, px_per_cm, blob_px=3.0):
    """Pixel sizes at the LEDs' height: the smallest gap between two lit LEDs,
    the step between neighbouring side-arm levels (l3) and whether lit LEDs
    stay apart (two LED images closer than about two blob radii merge)."""
    if fam.kind == "l3":
        gap = min(min(fam.side_cm), fam.arm_cm)
        lv = sorted(fam.side_cm)
        step = min(b - a for a, b in zip(lv, lv[1:])) if len(lv) > 1 else float("nan")
        return dict(min_gap_px=gap * px_per_cm, level_step_px=step * px_per_cm, resolved=gap * px_per_cm > 2 * blob_px)
    return dict(min_gap_px=fam.pitch_cm * px_per_cm, lit_gap_px=2 * fam.pitch_cm * px_per_cm,
                resolved=2 * fam.pitch_cm * px_per_cm > 2 * blob_px)
