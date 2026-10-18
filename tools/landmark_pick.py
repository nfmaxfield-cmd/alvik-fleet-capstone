"""Landmarks (boards, bricks, cinderblocks) picked by hand on the overhead video,
turned into floor positions with the floor calibration. Used by the Landmarks
mode of tools/calibrate_floor.py's page and by its --landmarks-from option.

Picks file (landmarks_picked.json, beside floor_calibration.json; also the
input of --landmarks-from):

    {"version": 1, "landmarks": [
      {"name": "A", "kind": "board", "clicks": [[u, v], [u, v]], "frame": 120,
       "thickness_cm": 2.0, "height_cm": null, "fix": [null, "36 3/32, 12 1/2"]},
      {"name": "cinderblock", "kind": "block", "clicks": [[u, v], [u, v], [u, v], [u, v]],
       "clicked_at": "top", "height_cm": 19.4, "fix": [null, null, null, null]}]}

- clicks: image pixels (origin top-left, pixel centres on whole numbers).
  board: the two ends of its FOOT (the line where it meets the floor, on the
  side the camera sees). block: 3-4 corners of its footprint in order around
  it, or 3 corners in a row (the 4th is completed), or 2 opposite corners (a
  rectangle lined up with the floor axes). clicked_at "top": the corners of
  its top face were clicked; they are moved down to the floor with height_cm
  and the camera height (the sides must be vertical).
- fix: per click, a tape-measured floor position that replaces the click:
  "x, y" in inches (36 3/32 or 36.09), or ending in "cm", or [x_cm, y_cm].
  For a board it is the same point that was clicked (foot end on the camera's
  side); for a block, the footprint corner on the floor.
- Board centre line: the camera sees the board's face toward the point below
  the lens (nadir), so the clicked foot is that face; the centre line is
  thickness_cm / 2 further from the nadir. Which face the robot ranges is left
  to the fusion (tools/fusion/landmarks.py): every face is written.
"""
import csv
import datetime as dt
import json
import math
import os
import re
import shutil
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if os.path.join(HERE, "fusion") not in sys.path:
    sys.path.insert(0, os.path.join(HERE, "fusion"))
import landmarks as LM  # noqa: E402  (tools/fusion/landmarks.py)

SOURCE = "user-picked"
DEFAULT_THICKNESS_CM = 2.0
CSV_COLS = ["name", "x1_cm", "y1_cm", "x2_cm", "y2_cm", "landmark", "kind", "source", "thickness_cm",
            "height_cm", "tape_fixed"]
TAPE_WARN_CM = 2.0          # click and tape differ by more than this: say so
NADIR_CLOSE_CM = 3.0        # nadir this close to a board's line: which face the camera sees is unclear


def _cf():
    import calibrate_floor as cf
    return cf


# ------------------------------------------------------------------ inches
_INCH_RE = re.compile(r"^(-)?\s*(?:(\d+(?:\.\d+)?)(?:\s*[\s-]\s*(\d+)\s*/\s*(\d+))?|(\d+)\s*/\s*(\d+))$")


def inches(text):
    """'36 3/32', '36-3/32', '36.09', '3/8', '-1 1/2' -> inches; anything else -> None
    (same rules as tools/run_sheet.py: a typo is refused, not guessed)."""
    t = str(text).strip().replace('"', "").strip()
    if t.lower().endswith("in"):
        t = t[:-2].strip()
    m = _INCH_RE.match(t)
    if not m:
        return None
    neg, whole, a, b, fa, fb = m.groups()
    if whole is not None:
        v = float(whole)
        if a is not None:
            if int(b) == 0 or int(a) >= int(b):
                return None
            v += int(a) / int(b)
    else:
        if int(fb) == 0 or int(fa) >= int(fb):
            return None
        v = int(fa) / int(fb)
    return -v if neg else v


def to_in(cm):
    """cm -> inches to the nearest 1/16, written like 15 3/4"."""
    n = round(abs(cm) / 2.54 * 16)
    w, f = divmod(n, 16)
    g = math.gcd(f, 16)
    frac = f" {f // g}/{16 // g}" if f else ""
    return f"{'-' if cm < 0 and n else ''}{w}{frac}\""


def parse_tape(v):
    """A tape position -> (x_cm, y_cm). Text in inches "36 3/32, 12 1/2" (comma
    between x and y), or "40.2, 31.75 cm", or a list [x_cm, y_cm]. Raises
    ValueError with a plain message."""
    if isinstance(v, (list, tuple)):
        if len(v) != 2:
            raise ValueError("needs two numbers: x, y")
        return float(v[0]), float(v[1])
    t = str(v).strip()
    cm = t.lower().endswith("cm")
    if cm:
        t = t[:-2]
    parts = [p for p in t.split(",") if p.strip()]
    if len(parts) != 2:
        raise ValueError('type x and y with a comma between them, in inches, e.g. 36 3/32, 12 1/2')
    if cm:
        try:
            return float(parts[0]), float(parts[1])
        except ValueError:
            raise ValueError("not two numbers in cm")
    a, b = inches(parts[0]), inches(parts[1])
    if a is None or b is None:
        raise ValueError(f'could not read "{parts[0 if a is None else 1].strip()}" as inches '
                         '(write whole inches and a fraction, e.g. 36 3/32, or a decimal, 36.09)')
    return a * 2.54, b * 2.54


# ------------------------------------------------------------------ calibration
def load_calibration(path):
    """(calibration dict, None) or (None, plain message why not)."""
    if not path or not os.path.isfile(path):
        return None, (f"No floor calibration yet ({path} is missing). Landmarks need it to turn your clicks into "
                      "floor positions: make one first in the Floor points mode and press Save calibration, "
                      "then come back. Your landmark clicks are kept meanwhile.")
    try:
        with open(path, encoding="utf-8-sig") as f:
            cal = json.load(f)
        np.linalg.inv(np.asarray(cal["H_floor_to_px"], float))
    except (OSError, ValueError, KeyError, TypeError, np.linalg.LinAlgError) as e:
        return None, f"The floor calibration {path} can't be read ({e})."
    return cal, None


def nadir_of(cal):
    """Floor point below the lens (cm): the calibration's, else estimated from it."""
    cam = cal.get("camera") or {}
    if cam.get("nadir_floor_cm"):
        return np.asarray(cam["nadir_floor_cm"], float)
    cf = _cf()
    return np.asarray(cf.estimate_nadir(np.asarray(cal["H_floor_to_px"], float),
                                        cal.get("image_size") or [640, 480], cam.get("height_cm"))[0], float)


def _px(cal, xy, z=0.0):
    u, v = _cf().project(cal, float(xy[0]), float(xy[1]), z)
    return [round(float(u), 2), round(float(v), 2)]


# ------------------------------------------------------------------ geometry
def block_footprint(pts):
    """Footprint corners (cm) from 2 opposite corners (rectangle along the floor
    axes), 3 corners in a row (the 4th completed) or 4 corners."""
    p = [np.asarray(q, float) for q in pts]
    if len(p) == 2:
        (x0, y0), (x1, y1) = np.minimum(p[0], p[1]), np.maximum(p[0], p[1])
        return [np.array(c) for c in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
    if len(p) == 3:
        return [p[0], p[1], p[2], p[0] + p[2] - p[1]]
    if len(p) == 4:
        c = np.mean(p, 0)
        return sorted(p, key=lambda q: math.atan2(q[1] - c[1], q[0] - c[0]))
    raise ValueError("a block needs 2, 3 or 4 corners")


def board_rect(p1, p2, t):
    """Corners of a board's footprint around its centre line (as tools/fusion/landmarks.py)."""
    p1, p2 = np.asarray(p1, float), np.asarray(p2, float)
    u = (p2 - p1) / np.hypot(*(p2 - p1))
    n = np.array([-u[1], u[0]]) * t / 2.0
    return [p1 + n, p2 + n, p2 - n, p1 - n]


def compute(cal, lm, area=None):
    """One landmark: floor positions of its clicks (with tape fixes), its board or
    block, its faces (as the fusion builds them) and pixel outlines to draw.
    cal may be None (then only the clicks are known)."""
    kind = lm.get("kind", "board")
    name = str(lm.get("name") or "").strip()
    clicks = [list(map(float, c)) for c in (lm.get("clicks") or [])]
    fixes = list(lm.get("fix") or [])
    fixes += [None] * (len(clicks) - len(fixes))
    height = lm.get("height_cm")
    height = float(height) if height not in (None, "") else None
    thick = lm.get("thickness_cm")
    thick = float(thick) if thick not in (None, "") else DEFAULT_THICKNESS_CM
    at_top = kind == "block" and lm.get("clicked_at") == "top"
    out = dict(name=name, kind=kind, ok=False, errors=[], warnings=[], ends=[], faces=[], outline=None,
               top_outline=None, board=None, obstacle=None, length_cm=None, height_cm=height)
    need = 2 if kind == "board" else (2, 3, 4)
    if not name:
        out["errors"].append("give it a name")
    if kind == "board" and thick <= 0:
        out["errors"].append("thickness must be more than 0")
    if cal is None:
        out["errors"].append("no floor calibration")
        return out
    cam = cal.get("camera") or {}
    z = 0.0
    if at_top:
        if not height:
            out["errors"].append("you clicked the top corners: type the block's height (tape, floor to top)")
        elif not cam.get("height_cm"):
            out["errors"].append("the floor calibration has no camera height, so top corners can't be moved down "
                                 "to the floor: click the floor corners instead")
        else:
            z = height
    for i, (u, v) in enumerate(clicks):
        x, y = _cf().to_floor(cal, u, v, z)
        e = dict(u=u, v=v, click_x=round(float(x), 2), click_y=round(float(y), 2), x=round(float(x), 2),
                 y=round(float(y), 2), tape=None, tape_error=None, diff_cm=None)
        f = fixes[i]
        if f not in (None, ""):
            try:
                tx, ty = parse_tape(f)
                e.update(x=round(tx, 2), y=round(ty, 2), tape=f, diff_cm=round(math.hypot(tx - x, ty - y), 2))
                e["tape_px"] = _px(cal, (tx, ty), z)
                if e["diff_cm"] > TAPE_WARN_CM:
                    out["warnings"].append(f"end {i + 1}: the tape and your click are {e['diff_cm']:.1f} cm apart. "
                                           "Check the tape reading and that you clicked the same spot; if both are "
                                           "right, the floor calibration is off here.")
            except ValueError as err:
                e["tape_error"] = str(err)
                out["errors"].append(f"end {i + 1}: tape entry not used: {err}")
        for k in ("x", "y", "click_x", "click_y"):
            e[k + "_in"] = to_in(e[k])
        out["ends"].append(e)
        if area and not (area[0] - 5 <= e["x"] <= area[2] + 5 and area[1] - 5 <= e["y"] <= area[3] + 5):
            out["warnings"].append(f"end {i + 1} is off the floor area ({e['x']:.0f}, {e['y']:.0f} cm)")
    pts = [np.array([e["x"], e["y"]]) for e in out["ends"]]
    if kind == "board":
        if len(pts) != 2:
            out["errors"].append(f"click both ends of the board's foot ({len(pts)} of 2 so far)")
            return out
        a, b = pts
        L = float(np.hypot(*(b - a)))
        out["length_cm"] = round(L, 2)
        out["length_in"] = to_in(L)
        if L < 1.0:
            out["errors"].append("the two ends are less than 1 cm apart")
            return out
        u = (b - a) / L
        n = np.array([-u[1], u[0]])
        nad = nadir_of(cal)
        side = float((nad - a) @ n)               # > 0: the nadir (and the face the camera sees) is to the left
        if abs(side) < NADIR_CLOSE_CM:
            out["warnings"].append("the camera is almost straight above this board, so it's unclear which side's "
                                   "foot you clicked: the board is placed centred between them (check by tape)")
            shift = np.zeros(2)
        else:
            shift = -np.sign(side) * n * thick / 2.0   # centre line: half a thickness away from the nadir
        p1, p2 = a + shift, b + shift
        out["board"] = dict(name=name, p1=[round(float(v), 2) for v in p1], p2=[round(float(v), 2) for v in p2],
                            thickness_cm=round(thick, 2))
        corners = board_rect(p1, p2, thick)
        faces = LM.faces_from_map(dict(boards=[out["board"]], obstacles=[]))
    else:
        if len(pts) < 2:
            out["errors"].append(f"click 2 to 4 corners ({len(pts)} so far)")
            return out
        if len(pts) > 4:
            out["errors"].append("a block has at most 4 corners")
            return out
        corners = block_footprint(pts)
        area_cm2 = 0.5 * abs(sum(corners[i][0] * corners[(i + 1) % 4][1] - corners[(i + 1) % 4][0] * corners[i][1]
                                 for i in range(4)))
        if area_cm2 < 1.0:
            out["errors"].append("the corners are (nearly) on one line")
            return out
        if len(pts) == 2:
            out["warnings"].append("2 corners: the block is taken as lined up with the floor's taped axes. "
                                   "If it is turned, click 3 corners instead.")
        out["obstacle"] = dict(id=name, shape="polygon", source="user",
                               points=[[round(float(c[0]), 2), round(float(c[1]), 2)] for c in corners])
        sides = [float(np.hypot(*(corners[(i + 1) % 4] - corners[i]))) for i in range(4)]
        out["sides_cm"] = [round(s, 1) for s in sides]
        out["sides_in"] = [to_in(s) for s in sides]
        faces = LM.faces_from_map(dict(boards=[], obstacles=[out["obstacle"]]))
    out["faces"] = [[f.name, [round(float(v), 2) for v in f.a], [round(float(v), 2) for v in f.b]] for f in faces]
    out["outline"] = [_px(cal, c, 0.0) for c in corners]
    if height and cam.get("height_cm") and height < 0.9 * float(cam["height_cm"]):
        out["top_outline"] = [_px(cal, c, height) for c in corners]
    out["ok"] = not out["errors"]
    return out


def compute_all(cal, picks, area=None):
    """All landmarks: list of compute() results, with duplicate names flagged."""
    res = [compute(cal, lm, area) for lm in picks.get("landmarks", [])]
    seen = {}
    for r in res:
        if r["name"]:
            seen.setdefault(r["name"].lower(), []).append(r)
    for rs in seen.values():
        if len(rs) > 1:
            for r in rs:
                r["errors"].append(f'two landmarks are called "{r["name"]}": rename one')
                r["ok"] = False
    return res


# ------------------------------------------------------------------ files
def load_picks(path):
    if path and os.path.isfile(path):
        with open(path, encoding="utf-8-sig") as f:
            d = json.load(f)
        if isinstance(d, list):
            d = dict(landmarks=d)
        d.setdefault("landmarks", [])
        return d
    return dict(version=1, landmarks=[])


def save_picks(path, picks):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(dict(picks, version=1, saved=dt.datetime.now().isoformat(timespec="seconds")), f, indent=1)
    os.replace(tmp, path)


def _landmark_of_row(r):
    if r.get("landmark"):
        return r["landmark"].strip()
    return re.sub(r"\s+(left|right|end p1|end p2|edge \d+)$", "", (r.get("name") or "").strip())


def write_csv(results, path):
    """landmarks.csv (tools/fusion/landmarks.py reads name,x1_cm,y1_cm,x2_cm,y2_cm;
    the other columns say where each row came from). Rows already in the file
    from another source are kept unless a picked landmark has the same name;
    the old file is kept as .bak. Returns messages."""
    msgs, keep = [], []
    names = {r["name"].lower() for r in results}
    if os.path.isfile(path):
        with open(path, newline="", encoding="utf-8-sig") as f:
            old = list(csv.DictReader(f))
        shutil.copy2(path, path + ".bak")
        replaced = set()
        for r in old:
            lmk = _landmark_of_row(r)
            if (r.get("source") or "") == SOURCE or lmk.lower() in names:
                replaced.add(lmk)
                continue
            keep.append(r)
        if keep:
            msgs.append(f"kept {len(keep)} face(s) already in {os.path.basename(path)} from elsewhere "
                        f"({', '.join(sorted({_landmark_of_row(r) for r in keep}))})")
        msgs.append(f"previous file kept as {os.path.basename(path)}.bak")
    rows = []
    for r in keep:
        rows.append({k: v for k, v in r.items() if k})
    for r in results:
        nfix = sum(1 for e in r["ends"] if e.get("tape") not in (None, ""))
        for nm, a, b in r["faces"]:
            rows.append(dict(name=nm, x1_cm=f"{a[0]:.2f}", y1_cm=f"{a[1]:.2f}", x2_cm=f"{b[0]:.2f}", y2_cm=f"{b[1]:.2f}",
                             landmark=r["name"], kind=r["kind"], source=SOURCE,
                             thickness_cm=r["board"]["thickness_cm"] if r["board"] else "",
                             height_cm=r.get("height_cm") or "", tape_fixed=nfix))
    cols = list(CSV_COLS)
    for r in rows:
        cols += [k for k in r if k not in cols]
    tmp = path + ".tmp"
    with open(tmp, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols, restval="")
        w.writeheader()
        w.writerows(rows)
    os.replace(tmp, path)
    return msgs


def update_map(results, picks, path):
    """Put the picked boards and blocks into a fleet map.json: a board replaces the
    board of the same name (its docking side and dock_at are kept), a block
    replaces the obstacle of the same id (polygon, source user, note says
    user-picked); blocks picked earlier and since deleted on the page are taken
    out. The old file is kept as .bak. Returns messages."""
    fm = LM._fleetmap()
    with open(path, encoding="utf-8-sig") as f:
        raw = json.load(f)
    fm.normalize(raw)                          # refuse to touch a broken map
    frames = {str(lm.get("name", "")).strip(): lm.get("frame") for lm in picks.get("landmarks", [])}
    boards = list(raw.get("boards") or [])
    obst = list(raw.get("obstacles") or [])
    picked_blocks = {r["name"] for r in results if r["obstacle"]}
    gone = [o.get("id") for o in obst if str(o.get("note", "")).startswith(SOURCE) and o.get("id") not in picked_blocks]
    obst = [o for o in obst if o.get("id") not in gone]
    nb = nbn = no = 0
    for r in results:
        if r["board"]:
            old = next((b for b in boards if b.get("name") == r["name"]), None)
            b = dict(r["board"], side=(old or {}).get("side", "auto"), dock_at=(old or {}).get("dock_at", 0.5))
            if old:
                boards[boards.index(old)] = b
                nb += 1
            else:
                boards.append(b)
                nbn += 1
        elif r["obstacle"]:
            o = dict(r["obstacle"], time=dt.datetime.now().isoformat(timespec="seconds"),
                     note=f"{SOURCE} on the video (tools/calibrate_floor.py Landmarks)"
                          + (f", frame {frames.get(r['name'])}" if frames.get(r["name"]) is not None else ""))
            obst = [x for x in obst if x.get("id") != r["name"]] + [o]
            no += 1
    raw["boards"], raw["obstacles"] = boards, obst
    fm.normalize(raw)
    shutil.copy2(path, path + ".bak")
    fm.save(raw, path)
    msgs = [f"map {path}: {nb} board(s) updated, {nbn} added, {no} block(s) written"
            + (f", {len(gone)} removed ({', '.join(map(str, gone))})" if gone else "")
            + f"; previous map kept as {os.path.basename(path)}.bak"]
    return msgs


def save_all(picks, cal_path, csv_path, map_path=None, area=None):
    """Write landmarks.csv (and map.json). Returns dict(ok, messages, skipped)."""
    cal, err = load_calibration(cal_path)
    if cal is None:
        return dict(ok=False, error=err)
    res = compute_all(cal, picks, area)
    good = [r for r in res if r["ok"]]
    bad = [r for r in res if not r["ok"]]
    if not good:
        return dict(ok=False, error="nothing to save: no landmark is complete"
                    + ("" if not bad else " (" + "; ".join(f"{r['name'] or '?'}: {r['errors'][0]}" for r in bad) + ")"))
    msgs = write_csv(good, csv_path)
    msgs.insert(0, f"wrote {sum(len(r['faces']) for r in good)} faces of {len(good)} landmark(s) to {csv_path}")
    if bad:
        msgs.append("NOT saved (fix them first): " + "; ".join(f"{r['name'] or '?'}: {r['errors'][0]}" for r in bad))
    if map_path:
        try:
            msgs += update_map(good, picks, map_path)
        except (OSError, ValueError) as e:
            msgs.append(f"map {map_path} NOT updated: {e}")
    return dict(ok=True, messages=msgs, csv=os.path.abspath(csv_path), results=res)
