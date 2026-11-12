#!/usr/bin/env python3
"""Discovery at the start of every run (e10, Phase C1, 7 Nov; Nick: one pipeline, every site; "observe, then trust").

Before any free move, each robot makes a short PROOF DRIVE on its spot, slowly, its own edge sensors on:
    stand 1.5 s  ->  turn +45 deg  ->  forward 12 cm  ->  stand  ->  turn +90 deg  ->  forward 12 cm  ->  stand
(a left-hand "L", about 17 cm from where it started; robots stand at least 40 cm apart at the start). The camera
watches; the robot reports its own pose (wheels + gyro). From the three standing spots (each the AVERAGE of the camera's
sightings while it stands) and its own three poses:

  camera found      every robot: is it seen at all (no robot seen: no camera, the run narrows to the robots' own
                    sensors; one robot not seen: it is out of the camera's view, it stays parked and is named)
  handedness        the camera's path turns LEFT where the robot's own gyro says it turned left. The other way: the
                    picture is mirrored (a wrong camera name: another camera's calibration). Before e10 this was found
                    only by the camera watch, mid-run, after bodies had gone over an edge (e09 rehearsal g2)
  scale             the camera's distances against the wheels' (median over robots; one robot's wheel error averages
                    out over several): off by more than SCALE_TOL: the board's size, which the camera's floor
                    calibration was made with, is wrong (or it is another camera's calibration). The board's real size
                    = the typed size / that scale: DISCOVERED, the typed one kept as the starting guess
  other names       when the picture as configured fails, the other calibrations the Pi has (mirrored, the other
                    camera) are tried on the same drive; the first that passes is used, and said
  tag heading       (roof tags) each robot's tag heading against the direction its camera spot travelled: a tag turned
                    on its robot (90, 180 deg) is found and corrected for that robot (tag_bridge's yaw_off)
  clock skew        when the camera's spot starts moving against when the robot's own pose did: the camera's stamps'
                    offset from the Pi's clock (the controller's align_clock refines it during the run)
  calibration       each robot's wheel scale (camera distance / own distance, the camera's scale taken out) and gyro
                    scale (the camera's turn between the two legs / its own): kept on the Pi keyed by the robot's name
                    (pooled over runs, with how sure each is) and saved on the robot itself (firmware 2026-11-05a's
                    `corr save`, no reflash) once the pooled value is sure enough

Everything here is a starting guess checked against evidence (CLAUDE.md "observe, then trust"); every number below
says whether it is chosen or measured. The switch (keep features, switch them off): MIRLT_DISCOVERY=0, or the runners'
--no-discovery: no proof drive, the camera and the typed board taken as configured, as before e10.
"""
import json
import math
import os
import time

VERSION = "start-discovery-2026-11-12e19"   # e19 (audit H2): each pooled row records its source (dry / real) and run. Before: "start-discovery-2026-11-09e11"   # e11: stands wait for the camera's sightings (STAND_MAX_S)
ENABLED = os.environ.get("MIRLT_DISCOVERY", "1") != "0"

TURN1_DEG = 45.0          # CHOSEN: the first turn (on its spot): a small one first
TURN2_DEG = 90.0          # CHOSEN: the turn between the legs (a quarter turn: the handedness and the gyro fit)
LEG_CM = 12.0             # CHOSEN: each leg. Longer legs fit better (the camera spot's noise / the leg), shorter ones
                          # need less room; the front edge sensors guard each leg. DOE: yes (fit spread vs room needed).
                          # 10 cm gave one false "board size wrong" in 5 simulator seeds
LEG_V = 4.0               # CHOSEN: cm/s (slow: the edge sensors' stop distance is small)
STAND_S = (1.5, 1.2, 1.2) # CHOSEN: standing before the first turn (the gyro's bias is taken there too) and after each leg
TIMEOUT_S = 45.0          # CHOSEN: the whole proof drive gives up after this (a robot that does not answer: named)
SCALE_TOL = 0.06          # CHOSEN: the camera's scale against the wheels' may be off by this before the board's size is
                          # called wrong (wheel scales differ +-3 % robot to robot: MEASURED 0.979-1.008 on 5 Nov), and
                          # only when it is also 3 standard errors from 1 (its spread over the robots)
SEEN_MIN = 4              # CHOSEN: sightings needed at a stand for it to count
STAND_MAX_S = 6.0         # (e11) a stand waits up to this long for SEEN_MIN sightings (CHOSEN: a camera at 1 fix/s still
                          # fills it; the 9 Nov home Pi gave 4-5 fixes/s, and 1.2 s stands got 3-4: "out of view")
TAG_TURN_DEG = 20.0       # CHOSEN: a tag's heading off its travel by more than this is a turned tag
SKEW_MIN_S = 0.15         # CHOSEN: a clock offset under this is left to align_clock (the picture's own ~0.15 s)
CAL_SURE = 0.01           # CHOSEN: a robot's pooled wheel / gyro scale is saved on the robot when its standard error is
                          # under this (1 %): one run's fit is +-1.5-3 %, so after a few runs
CAL_MAX_DEV = 0.12        # as corr save's limits: a fit further than this from 1 is refused (a misread, not a robot)


def _mean(v):
    return sum(v) / len(v) if v else None


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def circ_mean_deg(a):
    if not a:
        return None
    s = sum(math.sin(math.radians(x)) for x in a)
    c = sum(math.cos(math.radians(x)) for x in a)
    return math.degrees(math.atan2(s, c))


# ------------------------------------------------------------------------------------------------- the proof drive
class ProofDriver:
    """Runs the proof drive on every robot at once (each on its own spot), over any transport: send(name, text) sends a
    robot command; feed it on_follow / on_step / on_sighting / on_raw; call tick(t) often. done() when every robot has
    finished, failed, or the time ran out. Sightings must carry the robot's name (the start's find names them; roof
    tags name themselves)."""

    def __init__(self, names, send, t0=0.0, first_id=1, board=None, edge_cm=None):
        """first_id: the step ids start here (a robot refuses an id at or under the last it took: the controller then
        starts above data()[name]["last_id"]). board (x0, y0, x1, y1) and edge_cm: (e10, the 7 Nov run 2 incident d)
        a robot the camera puts less than edge_cm inside the board during its first stand does not drive (it is
        over or at an edge: data()[name]["at_edge"] says how far in it is); no board: every robot drives, as before."""
        self.names = list(names)
        self.board = tuple(board) if board else None
        self.edge_cm = edge_cm
        self.send = send
        self.t0 = t0
        self.R = {n: dict(phase=0, sid=None, own=[], cam=[], marks=[], stand_t=None, failed=None, next_id=first_id,
                          done=False, sent_t=None) for n in self.names}
        self.plan = [("stand", STAND_S[0]), ("T", TURN1_DEG), ("D", LEG_CM), ("stand", STAND_S[1]), ("T", TURN2_DEG),
                     ("D", LEG_CM), ("stand", STAND_S[2])]
        self.t = t0

    def on_follow(self, name, t, x, y, th, state="", last_done=None, last_failed=None):
        """A FOLLOW row: its own pose; last_done / last_failed (the ids FOLLOW carries) stand in for a lost report."""
        r = self.R.get(name)
        if r is not None:
            r["own"].append((float(t), float(x), float(y), float(th), str(state)))
            if r["sid"] is not None and last_done is not None and int(last_done) >= r["sid"]:
                self.on_step(name, r["sid"], "done")
            elif r["sid"] is not None and last_failed is not None and int(last_failed) >= r["sid"]:
                self.on_step(name, r["sid"], "failed", "(seen in FOLLOW)")

    def on_sighting(self, name, t, x, y, th=None, src="camera"):
        r = self.R.get(name)
        if r is not None:
            r["cam"].append((float(t), float(x), float(y), None if th is None else float(th), src))

    def on_step(self, name, sid, state, info=""):
        r = self.R.get(name)
        if r is None or r["sid"] is None or int(sid) != r["sid"]:
            return
        if state == "done":
            r["sid"] = None
            r["phase"] += 1
            r["stand_t"] = None
        elif state in ("failed", "bad", "full"):
            r["failed"] = "%s: %s" % (self.plan[r["phase"]][0], info)
            r["done"] = True
            r["sid"] = None

    def _own_now(self, r):
        return r["own"][-1] if r["own"] else None

    HEARTBEAT_S = 0.5        # the robots' watchdog stops a step after 1.5 s without a packet (firmware 11-05b)

    def tick(self, t):
        self.t = t
        for n, r in self.R.items():
            if r["done"]:
                continue
            if t - r.get("hb_t", -1e9) >= self.HEARTBEAT_S:
                r["hb_t"] = t
                self.send(n, "status")
            if r["phase"] >= len(self.plan):
                r["done"] = True
                continue
            kind, val = self.plan[r["phase"]]
            o = self._own_now(r)
            if kind == "stand":
                if r["stand_t"] is None:
                    r["stand_t"] = t
                    r["marks"].append(dict(phase=r["phase"], t0=t, t1=None))
                elif t - r["stand_t"] >= val and (t - r["stand_t"] >= STAND_MAX_S or sum(
                        1 for q in r["cam"] if r["stand_t"] + 0.2 <= q[0] <= t) >= SEEN_MIN):
                    # (e11, 9 Nov home: every robot "out of view" with the camera tracking all three) a stand ends when
                    # the camera has given it enough sightings (they arrive late and, on a busy Pi, 4-5 a second), not
                    # on the clock alone
                    r["marks"][-1]["t1"] = t
                    r["phase"] += 1
                    r["stand_t"] = None
                    if r["phase"] == 1 and self.board is not None and self.edge_cm is not None and \
                            len(r["cam"]) >= 3:
                        xs = sorted(q[1] for q in r["cam"])
                        ys = sorted(q[2] for q in r["cam"])
                        x, y = xs[len(xs) // 2], ys[len(ys) // 2]
                        b = self.board
                        e = min(x - b[0], b[2] - x, y - b[1], b[3] - y)
                        if e < self.edge_cm:
                            r["at_edge"] = round(e, 1)
                            r["failed"] = "at the edge: its middle %.1f cm from the board's edge by the camera" % e
                            r["done"] = True
                continue
            if r["sid"] is not None:
                if t - r["sent_t"] > 1.0 and o is not None and t - r["sent_t"] < 20.0:
                    self.send(n, r["text"])          # resent (a lost packet; the robot drops a duplicate id)
                    r["sent_t"] = t
                elif t - r["sent_t"] >= 20.0:
                    r["failed"] = "%s: no answer" % kind
                    r["done"] = True
                continue
            if o is None:
                continue
            sid = r["next_id"]
            r["next_id"] += 1
            if kind == "T":
                h1 = wrap180(o[3] + val)
                text = "fs %d T %.2f -" % (sid, h1)
            else:
                hr = math.radians(o[3])
                x1, y1 = o[1] + val * math.cos(hr), o[2] + val * math.sin(hr)
                text = "fs %d D %.2f %.2f %.2f %.2f %.2f %.2f 0 -" % (sid, o[1], o[2], x1, y1, o[3], LEG_V)
                r["onset_from"] = t
            r["sid"], r["text"], r["sent_t"] = sid, text, t
            self.send(n, text)
        if t - self.t0 > TIMEOUT_S:
            for r in self.R.values():
                if not r["done"]:
                    r["failed"] = r["failed"] or "timed out"
                    r["done"] = True

    def done(self):
        return all(r["done"] for r in self.R.values())

    def data(self):
        return {n: dict(own=r["own"], cam=r["cam"], marks=r["marks"], failed=r["failed"], last_id=r["next_id"] - 1,
                        at_edge=r.get("at_edge")) for n, r in self.R.items()}


# ------------------------------------------------------------------------------------------------- the analysis
def _stand_avg(rows, t0, t1, own=False):
    pts = [q for q in rows if t0 + 0.2 <= q[0] <= t1]
    if len(pts) < (2 if own else SEEN_MIN):
        return None
    x = _mean([q[1] for q in pts])
    y = _mean([q[2] for q in pts])
    ths = [q[3] for q in pts if q[3] is not None]
    return dict(x=x, y=y, th=circ_mean_deg(ths) if ths else None, n=len(pts))


def _onset(rows, t_from, ref, d_cm):
    """When the spot got d_cm from ref: a straight line fitted through its distance from ref over time where it was
    between 0.25 and 0.75 x 2 d_cm away (the leg's middle part, at a steady speed), solved for d_cm. Noisy sightings
    crossing early or late cancel out."""
    pts = []
    for q in rows:
        if q[0] < t_from:
            continue
        d = math.hypot(q[1] - ref[0], q[2] - ref[1])
        if d >= 1.6 * d_cm and pts:
            break
        if 0.5 * d_cm <= d <= 1.5 * d_cm:
            pts.append((q[0], d))
    if len(pts) < 4:
        return None
    mt = sum(p[0] for p in pts) / len(pts)
    md = sum(p[1] for p in pts) / len(pts)
    sxx = sum((p[0] - mt) ** 2 for p in pts)
    if sxx <= 1e-9:
        return None
    b = sum((p[0] - mt) * (p[1] - md) for p in pts) / sxx
    if b <= 0.5:
        return None
    return mt + (d_cm - md) / b


def mirror_x(board):
    m = board[0] + board[2]
    return lambda x, y: (m - x, y)


def analyse(data, board, candidates=None):
    """data: ProofDriver.data(). board: the typed board (the camera's floor calibration's). candidates: {name: f(x, y) ->
    (x, y)} other calibrations to try when the configured one fails (default: the picture mirrored left to right).
    Returns a dict (plain words in "say")."""
    cands = {"as configured": (lambda x, y: (x, y))}
    cands.update(candidates if candidates is not None else {"mirrored": mirror_x(board)})
    # the clock first: when each robot's camera spot crossed half a leg against when its own pose did (far above the
    # camera's noise; a calibration's scale changes it little); the camera's times are read with it before the
    # standing spots are averaged (a skewed clock otherwise cuts the legs short and fakes a wrong board size)
    skews = []
    for n, d in data.items():
        st = [m for m in d["marks"] if m.get("t1") is not None]
        for i in (0, 1):
            if len(st) <= i + 1:
                continue
            c0 = _stand_avg(d["cam"], st[i]["t0"], st[i]["t1"])
            o0 = _stand_avg(d["own"], st[i]["t0"], st[i]["t1"], own=True)
            c1 = _stand_avg(d["cam"], st[i + 1]["t0"] + 1.0, st[i + 1]["t1"] + 1.0)
            o1 = _stand_avg(d["own"], st[i + 1]["t0"], st[i + 1]["t1"], own=True)
            if c0 is None or o0 is None or c1 is None or o1 is None:
                continue
            # half of each leg as each source measured it (a camera reading 10 % long would otherwise cross early)
            dc = math.hypot(c1["x"] - c0["x"], c1["y"] - c0["y"])
            do = math.hypot(o1["x"] - o0["x"], o1["y"] - o0["y"])
            if dc < 0.5 * LEG_CM or do < 0.5 * LEG_CM:
                continue
            tc = _onset([(q[0], q[1], q[2]) for q in d["cam"]], st[i]["t1"] - 3.0, (c0["x"], c0["y"]), 0.5 * dc)
            to = _onset([(q[0], q[1], q[2]) for q in d["own"]], st[i]["t1"], (o0["x"], o0["y"]), 0.5 * do)
            if tc is not None and to is not None:
                skews.append(tc - to)
    skew = sorted(skews)[len(skews) // 2] if skews else 0.0
    if abs(skew) >= 0.05:
        data = {n: dict(d, cam=[(q[0] - skew,) + tuple(q[1:]) for q in d["cam"]]) for n, d in data.items()}
    robots = {}
    for n, d in data.items():
        st = [m for m in d["marks"] if m.get("t1") is not None]
        own = [_stand_avg(d["own"], m["t0"], m["t1"], own=True) for m in st]
        cam = [_stand_avg(d["cam"], m["t0"], m["t1"]) for m in st]
        robots[n] = dict(failed=d["failed"], n_cam=len(d["cam"]), own=own, cam=cam, stands=len(st),
                         marks=st)
    seen = [n for n, r in robots.items() if r["n_cam"] >= SEEN_MIN]
    out = dict(version=VERSION, robots={}, seen=seen, unseen=[n for n in robots if n not in seen], camera="ok",
               calibration="as configured", scale=1.0, board=list(board), board_found=list(board), say=[],
               tag_yaw={}, clock_s=None, fits={})
    if not seen:
        out["camera"] = "missing"
        out["say"].append("No robot was seen by the camera: no camera (or the wrong one). The run narrows to the "
                          "robots' own sensors: only ground they prove by feel is used.")
        return out
    full = {n: r for n, r in robots.items() if r["stands"] >= 3 and all(c is not None for c in r["cam"][:3]) and
            all(o is not None for o in r["own"][:3])}
    if not full:
        out["camera"] = "unproven"
        out["say"].append("No robot finished its proof drive in view of the camera (%s): the camera is not proven "
                          "by the drive." % ", ".join(
                              "%s %s" % (n, r["failed"] or "out of view") for n, r in robots.items()))
        return out

    def verdict(f):
        hand, ratio, onb, legs = [], [], 0, {}
        for n, r in full.items():
            P = [f(c["x"], c["y"]) for c in r["cam"][:3]]
            O = [(o["x"], o["y"]) for o in r["own"][:3]]
            v1, v2 = (P[1][0] - P[0][0], P[1][1] - P[0][1]), (P[2][0] - P[1][0], P[2][1] - P[1][1])
            w1, w2 = (O[1][0] - O[0][0], O[1][1] - O[0][1]), (O[2][0] - O[1][0], O[2][1] - O[1][1])
            cc = v1[0] * v2[1] - v1[1] * v2[0]
            co = w1[0] * w2[1] - w1[1] * w2[0]
            Lc = math.hypot(*v1) + math.hypot(*v2)
            Lo = math.hypot(*w1) + math.hypot(*w2)
            if Lo < 0.5 * LEG_CM:
                continue
            hand.append((cc > 0) == (co > 0))
            ratio.append(Lc / Lo)
            onb += int(all(board[0] - 5 <= p[0] <= board[2] + 5 and board[1] - 5 <= p[1] <= board[3] + 5 for p in P))
            legs[n] = (P, O, v1, v2, w1, w2)
        if not hand:
            return None
        k = sorted(ratio)[len(ratio) // 2]
        return dict(hand_ok=sum(hand) > len(hand) / 2.0, hand=hand, scale=k, ratio=ratio, on_board=onb, n=len(hand),
                    legs=legs)

    tried = []
    chosen = None
    for cname, f in cands.items():
        v = verdict(f)
        if v is None:
            continue
        tried.append((cname, v))
        if v["hand_ok"] and v["on_board"] == v["n"] and abs(v["scale"] - 1.0) <= 0.35:
            chosen = (cname, f, v)
            break
    if chosen is None:
        out["camera"] = "wrong"
        out["say"].append("The camera's picture did not match the robots' own movement under any calibration the Pi "
                          "has (%s): the camera is not trusted; the run narrows to the robots' own sensors." % "; ".join(
                              "%s: %s, scale %.2f" % (c, "turns the right way" if v["hand_ok"] else "mirrored",
                                                      v["scale"]) for c, v in tried))
        out["tried"] = [(c, dict(hand_ok=v["hand_ok"], scale=round(v["scale"], 3))) for c, v in tried]
        return out
    cname, f, v = chosen
    out["tried"] = [(c, dict(hand_ok=vv["hand_ok"], scale=round(vv["scale"], 3))) for c, vv in tried]
    out["calibration"] = cname
    if cname != "as configured":
        out["camera"] = "wrong name"
        out["say"].append("The camera's picture was %s against the robots' own turns: a wrong camera name. Using the %s "
                          "calibration, which matches." % ("mirrored" if not tried[0][1]["hand_ok"] else "off",
                                                          cname))
    k = v["scale"]
    out["scale"] = round(k, 4)
    rr = v["ratio"]
    k_se = max(0.01, (math.sqrt(sum((x - _mean(rr)) ** 2 for x in rr) / (len(rr) - 1)) / math.sqrt(len(rr)))
               if len(rr) > 1 else 0.03)
    out["scale_se"] = round(k_se, 4)
    if abs(k - 1.0) > max(SCALE_TOL, 3.0 * k_se):
        bf = [board[0], board[1], board[0] + (board[2] - board[0]) / k, board[1] + (board[3] - board[1]) / k]
        out["board_found"] = [round(q, 1) for q in bf]
        out["say"].append("The camera measures distances %.0f %% %s than the wheels do: the board's size it was set up "
                          "with is wrong. Board found: %.0f x %.0f cm (typed %.0f x %.0f): used from now on." % (
                              abs(k - 1) * 100, "longer" if k > 1 else "shorter", bf[2] - bf[0], bf[3] - bf[1],
                              board[2] - board[0], board[3] - board[1]))
    else:
        k = 1.0
    out["k_used"] = k

    def fix(x, y):
        a, b = f(x, y)
        return board[0] + (a - board[0]) / k, board[1] + (b - board[1]) / k
    out["fix"] = fix
    # per robot: where it is now, its heading, its tag's turn, its wheel and gyro fit
    for n, (P, O, v1, v2, w1, w2) in v["legs"].items():
        Pc = [fix(full[n]["cam"][i]["x"], full[n]["cam"][i]["y"]) for i in range(3)]
        d1 = math.degrees(math.atan2(Pc[1][1] - Pc[0][1], Pc[1][0] - Pc[0][0]))
        d2 = math.degrees(math.atan2(Pc[2][1] - Pc[1][1], Pc[2][0] - Pc[1][0]))
        own_turn = wrap180(math.degrees(math.atan2(w2[1], w2[0]) - math.atan2(w1[1], w1[0])))
        cam_turn = wrap180(d2 - d1)
        Lc = math.dist(Pc[0], Pc[1]) + math.dist(Pc[1], Pc[2])
        Lo = math.hypot(*w1) + math.hypot(*w2)
        wfit = Lc / Lo if Lo > 0 else None
        gfit = cam_turn / own_turn if abs(own_turn) > 30 else None
        # how sure: each stand's spot to ~sigma / sqrt(n) (0.5-1 cm sightings)
        sig = 0.7 / math.sqrt(max(1, min(c["n"] for c in full[n]["cam"][:3])))
        out["fits"][n] = dict(wheel=round(wfit, 4) if wfit else None, wheel_se=round(sig * 2.0 / max(1e-6, Lo), 4),
                              gyro=round(gfit, 4) if gfit else None,
                              gyro_se=round(math.degrees(sig * 2.4 / LEG_CM) / max(1.0, abs(own_turn)), 4))
        rob = dict(x=round(Pc[2][0], 2), y=round(Pc[2][1], 2), heading=round(d2, 1), cam_turn=round(cam_turn, 1),
                   own_turn=round(own_turn, 1))
        # the roof tag's heading against its travel (leg 2: the stand after it)
        ths = [c["th"] for c in full[n]["cam"][1:3] if c.get("th") is not None]
        if ths:
            # tag headings during the stand after each leg vs that leg's travel direction (the picture corrected)
            offs = []
            for i, dleg in ((1, d1), (2, d2)):
                th = full[n]["cam"][i].get("th")
                if th is not None:
                    dx, dy = f_inv_dir(f, math.cos(math.radians(th)), math.sin(math.radians(th)))
                    offs.append(wrap180(dleg - math.degrees(math.atan2(dy, dx))))
            off = circ_mean_deg(offs)
            rob["tag_off"] = round(off, 1) if off is not None else None
            if off is not None and abs(off) > TAG_TURN_DEG:
                q = round(off / 90.0) * 90.0 if abs(off - round(off / 90.0) * 90.0) < 15 else off
                out["tag_yaw"][n] = q
                out["say"].append("%s's roof tag is turned %.0f deg on the robot (its heading against where it drove): "
                                  "corrected for %s from now on." % (n, -q, n))
        out["robots"][n] = rob
    if skews:
        out["clock_s"] = round(skew, 3)
        if abs(skew) >= SKEW_MIN_S:
            out["say"].append("The camera's time stamps are %+.2f s from the Pi's clock (from when the robots started "
                              "to move): read with that shift from the start." % skew)
    for n in out["unseen"]:
        out["say"].append("%s was not seen by the camera on its spot: it stays parked (out of the camera's view)." % n)
    if out["camera"] == "ok" and not out["say"]:
        out["say"].append("Camera, board and clock check out against the robots' own movement.")
    return out


def f_inv_dir(f, dx, dy):
    """A direction (dx, dy) in the corrected picture's frame, carried through f's linear part (f is affine). Tags read
    their heading in the picture as configured; f maps the configured picture to the right one."""
    a0, b0 = f(0.0, 0.0)
    a1, b1 = f(dx, dy)
    return (a1 - a0, b1 - b0)


# ------------------------------------------------------------------------------------------------- calibration store
def _owner_default():
    try:
        import safety_group as SG_
        return SG_.value("MIRLT_WHEEL_OWNER")
    except Exception:  # noqa: BLE001
        return "pool"


def cal_path(conf_dir, robot):
    return os.path.join(os.path.expanduser(conf_dir), "robots", "%s.json" % robot)


def pool_fit(conf_dir, robot, fit, when=None, source=None, run=None, in_use=None):
    """Adds one run's fit (wheel, gyro with their standard errors) to the robot's history on the Pi and returns the
    pooled values (inverse-variance means) and whether each is sure enough to save on the robot. (e19, audit H2) source
    ("dry" / "real") and run (its folder's name) are kept on the row, so a dry run's row can be found later."""
    p = cal_path(conf_dir, robot)
    try:
        with open(p) as fh:
            d = json.load(fh)
    except (OSError, ValueError):
        d = dict(robot=robot, runs=[])
    if fit:
        row = dict(fit, t=when or time.strftime("%Y-%m-%d %H:%M:%S"))
        for k_, p_ in (("wheel", "wheel_scale"), ("gyro", "gyro_scale")):
            if in_use and in_use.get(p_) is not None and not fit.get("abs"):
                row[k_ + "_in_use"] = float(in_use[p_])     # (e19, audit 7) the fit is relative to the value in use
        if source:
            row["source"] = source
        if run:
            row["run"] = run
        d["runs"].append(row)
        d["runs"] = d["runs"][-30:]
    pooled = {}
    for k in ("wheel", "gyro"):
        # (e19, audit 7) pooled as ABSOLUTE scales: a row is absolute ("abs") or relative to the value in use when it was
        # measured (k_in_use; rows from before e19 carry none: taken as 1, the factory value)
        xs = [((r[k] if r.get("abs") else r[k] * float(r.get(k + "_in_use") or 1.0)), r.get(k + "_se") or 0.03)
              for r in d["runs"] if r.get(k) and abs(r[k] - 1.0) <= CAL_MAX_DEV]
        if not xs:
            continue
        w = [1.0 / max(1e-6, se * se) for _, se in xs]
        m = sum(x * wi for (x, _), wi in zip(xs, w)) / sum(w)
        se = math.sqrt(1.0 / sum(w))
        pooled[k] = dict(value=round(m, 4), se=round(se, 4), runs=len(xs), sure=se < CAL_SURE)
    d["pooled"] = pooled
    os.makedirs(os.path.dirname(p), exist_ok=True)
    with open(p + ".tmp", "w") as fh:
        json.dump(d, fh, indent=1)
    os.replace(p + ".tmp", p)
    return pooled


def corr_commands(pooled, current=None, owner=None):
    """The robot commands that save a sure pooled fit on the robot (firmware 2026-11-05a corr save: no reflash). current:
    the robot's present wheel_scale / gyro_scale (its M, line at power-on), multiplied by the fit. (e19, audit 7; owner
    "pool", the default) the pool is absolute: its value is saved only when the value in use was read back and the two
    differ by more than the pool's own 95 % error bar (1.96 se); a correction inside its error bar is never saved."""
    cur = current or {}
    out = []
    if (owner or _owner_default()) == "pool":
        for k, p_ in (("wheel", "wheel_scale"), ("gyro", "gyro_scale")):
            pk = pooled.get(k) or {}
            if not pk.get("sure") or cur.get(p_) is None:
                continue
            if abs(pk["value"] - float(cur[p_])) > 1.96 * pk["se"]:
                out.append("corr save %s %.4f" % (p_, pk["value"]))
        return out
    if pooled.get("wheel", {}).get("sure"):
        out.append("corr save wheel_scale %.4f" % (cur.get("wheel_scale", 1.0) * pooled["wheel"]["value"]))
    if pooled.get("gyro", {}).get("sure"):
        out.append("corr save gyro_scale %.4f" % (cur.get("gyro_scale", 1.0) * pooled["gyro"]["value"]))
    return out
