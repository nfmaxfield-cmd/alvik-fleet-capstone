#!/usr/bin/env python3
"""The hard simulator (2026-11-12): the fast free-roam simulator (freeroam_fastsim.Sim, unchanged) with a REALISM
PROFILE that reproduces what the real home runs show, from distributions MEASURED on their logs
(tools/analysis/realism_measure.py), plus a HARSH profile that pushes beyond them. CLAUDE.md "Observe, then trust",
point 5: the simulator deliberately breaks the guesses.

Profiles (switch: profile=..., or MIRLT_SIM_PROFILE for the batch tools):
  easy      the fast simulator exactly as before (12 pictures a second, fixes 0.12-0.2 s late, one Pi freeze a run,
            dropouts every ~8 min): kept, and the default of freeroam_fastsim itself
  measured  every number below drawn from a measured run's distributions (sim_profiles/measured-*.json, built by
            build_profile() from the measure's JSON):
              timing     (timing="replay", the default) the real run's own timeline, from a random point of it and
                         wrapping round: every named picture's time and delay, every Pi-wide silence (the controller
                         hears and sends nothing: two or more robots' quiet spells overlapping) and each robot's own
                         silences (WiFi). Fixes and robot messages wait in the queue during a silence and arrive
                         together, late, as on the Pi. This keeps the real run's timing second by second (a block
                         bootstrap whose block is the whole run). timing="draw": independent draws instead: pictures a
                         second every 10 s, each 30 s a real window's (10th, 50th, 90th percentile) delays, silences
                         at the measured rates and lengths (these over-count: a draw's late fix and a silence's wait add)
              sightings  ONE bridge sighting a robot a picture (as the home bridge sends: no names), seen in the
                         measured share of pictures, from "both" cameras / one camera in the measured shares, each with
                         its measured standing scatter; the side camera with its own offset; two bodies closer than
                         3 cm: the bridge withholds both (merged outline)
              dropouts   the robot missing from 2+ pictures in a row at the measured rate and lengths
              curves     each robot turns its curves by its measured ratio (Left 1.065 ...; others drawn from the pool)
              slips      wheels turning, body not, at the measured rate and lengths
              edge line  the camera's calibration off by the measured edge offset (a shift, random direction)
              edges      edge physics on (hang, tilt, fall)
  harsh     measured, then: latency x2, dropout bursts (x3 rate, x3 length, clustered), one robot HUNG (wheels spin,
            body still) for 60-180 s mid-board, calibration 5 cm off, a dark patch (sightings 30 %), the camera's
            clock 0.4 s skewed, one robot RESTARTING (silent 8-15 s, queue lost, then its own pose from the camera with
            its heading from a fresh gyro: anywhere), WRONG camera names (the side camera's sightings through the other
            calibration: mirrored) for a spell, a MISSING camera (overhead gone for 10 min: side only, its scatter,
            the far quarter unseen). Each fault can be switched alone: faults={"hung": True, ...} on top of measured.

Measures (measure_sim) use the SAME definitions as realism_measure.py (picture_spells, quiet_spells), so the
validation compares like with like: hardsim_validate.py.

Version / changelog:
  fastsim-realism-2026-11-12a  first version (hard simulator build).
  fastsim-realism-2026-11-12e20  every sighting says its camera ("both" / "side", as the home bridge's cam_bridge:<camera>)
               and "dark" in the dark patch; cams=[...]: several cameras, each seeing every robot on its own, each breaking
               a guess of its own (a steep angle: noise stretched along its depth; a bump mid-run: its picture turned; a
               dark patch only it has; a third camera), and cam_swap=(t0, t1, a, b): two cameras' names swapped for a spell
               (each picture through the other's calibration: one mirrored left-right, the other top-bottom).
  fastsim-realism-2026-11-12e21  faults "bumps" (BUMPS_PER_H a robot-hour, each moving the body BUMP_CM one way, its
               wheels and own pose not: a push, a knock by another robot, a seam) and "slips" (SLIPS_PER_H spells of
               SLIP_S of fully stuck wheels: the body still while its wheels and own pose drive on), both added to harsh.
"""
import bisect
import json
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
_AN = os.path.join(os.path.dirname(HERE), "analysis")
if _AN not in sys.path:
    sys.path.insert(0, _AN)

VERSION = "fastsim-realism-2026-11-12e21"   # e21: two faults (Nick 12 Nov, decision 5): "bumps" (the robot is pushed: its body moves, its wheels do not) and "slips" (stuck wheels: spells of turning wheels and a still body), both in harsh; the simulated robot's own filter is freeroam_fastsim's firmware filter (the ratchet). Before: "fastsim-realism-2026-11-12e20"   # e20: cameras named on every sighting; cams=[...] and cam_swap (the simulator breaks the camera guesses). Before: "fastsim-realism-2026-11-12e19"   # e19: the restart model calls the runner's new restart hooks (on_restart, follow_again). Before: "fastsim-realism-2026-11-12a"
PROFILE_DIR = os.path.join(HERE, "sim_profiles")
DEFAULT_MEASURED = "measured-home-2026-11-11-0858.json"
PROFILES = ("easy", "measured", "harsh")
HARSH_FAULTS = ("latency2", "bursts", "hung", "calib5", "dark", "skew", "restart", "wrong_names", "missing_cam", "bumps",
                "slips")       # (e21) bumps and slips added
MERGE_GAP_CM = 3.0          # fastsim_sources.MERGE_GAP_CM (CHOSEN there)
QUIET_BASE_S = 0.2          # a stall of length L leaves a quiet spell of about L + the quiet when it began; this is
                            # that quiet's typical value (MEASURED: the quiet's 25th percentile 0.08 s on 11 Nov; set so
                            # the simulated quiet spells match the real ones: hardsim_validate.py)
NEVER_STILL_S = 5.0         # CHOSEN: a stand longer than this is counted as a never-still violation (the derived pause
                            # is about 1-1.5 s; 5 s leaves room for a reflex hold and one creep). DOE: no (a reporting bar)


# -- profiles --------------------------------------------------------------------------------------------------------
# (e21, 5) CHOSEN stresses (not estimates; DOE: no): the 11 Nov runs' measured event jumps run 2-10 cm (cone_fit.json:
# "felt a robot" p50 1.8, p95 8.1; "hazard" p95 9.9); a bump is drawn 3-10 cm. Rates: about 2 felt / front-stop events a
# robot-hour at home; the harsh profile triples that.
BUMPS_PER_H = 6.0
BUMP_CM = (3.0, 10.0)
SLIPS_PER_H = 6.0
SLIP_S = (2.0, 6.0)


def qsample(q, rng, scale=1.0):
    """A draw from a quantile summary (realism_measure.quant): piecewise linear between the stored quantiles."""
    pts = [(0.0, q.get("p5", q.get("p50")))]
    for k in ("p5", "p10", "p25", "p50", "p75", "p90", "p95", "p99"):
        if k in q:
            pts.append((int(k[1:]) / 100.0, q[k]))
    pts.append((1.0, q.get("max", pts[-1][1])))
    u = rng.random()
    for (a, va), (b, vb) in zip(pts, pts[1:]):
        if u <= b:
            f = 0.0 if b == a else (u - a) / (b - a)
            return (va + f * (vb - va)) * scale
    return pts[-1][1] * scale


def build_profile(m, name=None):
    """A measured profile from one run's measures (realism_measure.measure() output)."""
    st = m.get("stalls") or {}
    per_rh = st.get("per_robot_h", 0.0)
    sh = st.get("shared_pct", 0.0) / 100.0
    prof = dict(kind="measured", name=name or ("measured-" + m["run"]), source_run=m["run"], version=VERSION,
                hours=m.get("hours"),
                pictures_s=m["pictures_s"],
                delay_windows=m["delay_windows"],
                stall_pi_per_h=round(per_rh * sh, 3),            # each Pi-wide stall shows as one spell per robot
                stall_wifi_per_robot_h=round(per_rh * (1 - sh), 3),
                stall_len_s=st.get("length_s"),
                cam_gaps_per_robot_h=m["cam_gaps"]["per_robot_h"], cam_gap_len_s=m["cam_gaps"]["length_s"],
                seen_pct=m.get("seen_pct"), src_share=m.get("src_share"),
                standing_sd_cm={k: v.get("p50") for k, v in (m.get("standing_sd_cm") or {}).items() if v},
                curve_ratio={k: v["mean"] for k, v in (m.get("curve_ratio") or {}).items()},
                slips_per_robot_h=(m.get("slips") or {}).get("per_robot_h", 0.0),
                slip_len_s=(m.get("slips") or {}).get("length_s"),
                edge_offset_cm=m.get("edge_offset_cm"), timeline=m.get("timeline"))
    return prof


def load_profile(profile="measured", path=None):
    if profile == "easy":
        return dict(kind="easy", name="easy")
    p = path or os.path.join(PROFILE_DIR, DEFAULT_MEASURED)
    with open(p) as f:
        prof = json.load(f)
    if profile == "harsh":
        prof = dict(prof, kind="harsh", name="harsh (" + prof["name"] + ")")
    return prof


# -- the simulator ----------------------------------------------------------------------------------------------------
def make_class(F):
    """The hard simulator class over a freeroam_fastsim module F (so the same realism runs against any code version:
    hardsim_batch.py --code DIR puts another tools/fleet first)."""
    import tracking_layer as TL
    import freeroam_core as FC

    class HardSim(F.Sim):
        def __init__(self, seed, hours, profile="measured", prof=None, faults=None, timing="replay", **kw):
            self.prof = prof or load_profile(profile)
            self.timing = timing
            self.kind = self.prof["kind"]
            if self.kind == "easy" and not faults:
                super().__init__(seed, hours, **kw)
                self._real = False
                self._init_measures()
                return
            self._real = True
            self.cams_spec = kw.pop("cams", None)          # (e20) several cameras, each with its own faults
            self.cam_swap = kw.pop("cam_swap", None)       # (e20) (t0, t1, a, b): two cameras' names swapped
            fl = dict.fromkeys(HARSH_FAULTS, True) if self.kind == "harsh" else {}
            fl.update(faults or {})
            self.faults = {k: v for k, v in fl.items() if v}
            self.rrng = random.Random(seed * 4243 + 17)
            kw.setdefault("camera", "detections")
            kw.setdefault("edge_physics", True)
            kw.setdefault("sensors", True)
            kw.setdefault("dead_robot", False)
            kw.setdefault("pi_freeze", False)
            kw.setdefault("dropouts", False)
            if "skew" in self.faults:
                kw.setdefault("cam_clock", 0.4)
            # the camera's calibration: the measured edge offset (a shift in a random direction), or 5 cm (harsh)
            eo = self.prof.get("edge_offset_cm") or {}
            mag = 5.0 if "calib5" in self.faults else abs(eo.get("p50", 0.0) or 0.0)
            if mag > 0.05 and "cam_fault" not in kw:
                a = self.rrng.uniform(0, 2 * math.pi)
                kw["cam_fault"] = dict(shift=(mag * math.cos(a), mag * math.sin(a)))
            super().__init__(seed, hours, **kw)
            self._setup(hours)
            self._init_measures()

        def _delay_draw(self):
            """One picture's delay: its 30 s window's (10th, 50th, 90th percentile) from a real window, piecewise
            linear between them, a short tail below the 10th and an exponential tail above the 90th."""
            k = int(self.t // 30.0)
            w = self._dwin.get(k)
            if w is None:
                w = self._dwin[k] = self.rrng.choice(self.prof["delay_windows"])
            if len(w) == 2:                       # (median, p90) only
                w = (w[0] * 0.8, w[0], w[1])
            p10, p50, p90 = w
            u = self.rrng.random()
            if u < 0.1:
                d = p10 - (p50 - p10) * 0.5 * (0.1 - u) / 0.1
            elif u < 0.5:
                d = p10 + (p50 - p10) * (u - 0.1) / 0.4
            elif u < 0.9:
                d = p50 + (p90 - p50) * (u - 0.5) / 0.4
            else:
                d = p90 + self.rrng.expovariate(1.2816 / max(1e-3, p90 - p50))
            return max(0.05, d) * (2.0 if "latency2" in self.faults else 1.0)

        # -- set-up ----------------------------------------------------------------------------------------------
        def _setup(self, hours):
            P, rng, T = self.prof, self.rrng, hours * 3600.0
            self._dwin = {}
            self._fps = {}
            TL_ = P.get("timeline") if self.timing == "replay" else None
            self._replay = None
            if TL_ and TL_.get("pics"):
                # the real run's own timeline from a random point (wrapping round): its pictures (when, and each one's
                # delay), its Pi-wide silences and each robot's own; the timing is the real one, second by second
                span = float(TL_["span_s"])
                o = rng.uniform(0.0, span)
                pt = [q[0] for q in TL_["pics"]]
                self._replay = dict(o=o, span=span, pt=pt, pd=[q[1] for q in TL_["pics"]], i=bisect.bisect_left(pt, o))

                def tiled(spells):
                    out = []
                    for rep in range(int(T // span) + 2):
                        for a, b in spells:
                            a2, b2 = a - o + rep * span, b - o + rep * span
                            if b2 > 0 and a2 < T:
                                out.append((max(0.0, a2), b2))
                    return sorted(out)
                self.stalls = tiled(TL_.get("pi_stalls") or [])
                wf = TL_.get("wifi") or {}
                keys = sorted(wf)
                self.wifi = {n: tiled(wf.get(n) or (wf[keys[k % len(keys)]] if keys else []))
                             for k, n in enumerate(self.names)}
            # Pi-wide stalls and WiFi spells (drawn: timing "draw", or no timeline)
            if self._replay is None:
                self._draw_stalls(P, rng, T)
            self._after_stalls(P, rng, T)

        def _draw_stalls(self, P, rng, T):
            self.stalls = []
            rate = P.get("stall_pi_per_h", 0.0) / 3600.0
            t = rng.expovariate(rate) if rate > 0 else T + 1
            while t < T:
                L = max(0.2, qsample(P["stall_len_s"], rng) - QUIET_BASE_S)
                self.stalls.append((t, t + L))
                t += L + rng.expovariate(rate)
            self.wifi = {n: [] for n in self.names}
            rate = P.get("stall_wifi_per_robot_h", 0.0) / 3600.0
            for n in self.names:
                t = rng.expovariate(rate) if rate > 0 else T + 1
                while t < T:
                    L = max(0.2, qsample(P["stall_len_s"], rng) - QUIET_BASE_S)
                    self.wifi[n].append((t, t + L))
                    t += L + rng.expovariate(rate)

        def _after_stalls(self, P, rng, T):
            # camera dropouts (2+ pictures missed) at the measured rate and lengths; harsh: bursts
            burst = "bursts" in self.faults
            rate = P.get("cam_gaps_per_robot_h", 0.0) / 3600.0 * (3.0 if burst else 1.0)
            for n in self.names:
                g = [q for q in self.gaps[n] if not (q[1] - q[0] < 30)]      # keep the base's long injected gaps
                t = rng.expovariate(rate) if rate > 0 else T + 1
                while t < T:
                    # the measured length (picture before the gap to the one after) used as the time unseen: at the
                    # replayed picture rate this gives the real dropout rate (taking a picture off undercounts them)
                    L = qsample(P["cam_gap_len_s"], rng) * (3.0 if burst else 1.0)
                    g.append((t, t + L))
                    if burst and rng.random() < 0.5:             # clustered: another one right after
                        t2 = t + L + rng.uniform(0.5, 3.0)
                        L2 = qsample(P["cam_gap_len_s"], rng) * 3.0
                        g.append((t2, t2 + L2))
                        t = t2 + L2
                    else:
                        t += L
                    t += rng.expovariate(rate)
                self.gaps[n] = g
            # curves: each robot's turn ratio (its own measured one, else one from the pool)
            pool = list((P.get("curve_ratio") or {}).values()) or [1.0]
            for n, b in self.bots.items():
                k = (P.get("curve_ratio") or {}).get(n) or rng.choice(pool)
                b.gyro_err = k - 1.0
            # slips (wheels turn, body does not): measured rate and lengths; harsh: one robot HUNG 60-180 s
            self.slip = {n: [] for n in self.names}
            rate = P.get("slips_per_robot_h", 0.0) / 3600.0
            for n in self.names:
                t = rng.expovariate(rate) if rate > 0 else T + 1
                while t < T:
                    L = max(0.5, qsample(P["slip_len_s"], rng)) if P.get("slip_len_s") else 1.0
                    self.slip[n].append((t, t + L, rng.uniform(0.0, 0.3)))
                    t += L + rng.expovariate(rate)
            if "slips" in self.faults:                       # (e21, 5) stuck wheels: spells with the body still
                for n in self.names:
                    t = rng.expovariate(SLIPS_PER_H / 3600.0)
                    while t < T:
                        L = rng.uniform(*SLIP_S)
                        self.slip[n].append((t, t + L, 0.0))
                        t += L + rng.expovariate(SLIPS_PER_H / 3600.0)
            self.bumps = {n: [] for n in self.names}
            if "bumps" in self.faults:                       # (e21, 5) pushes: the body moves, its wheels do not
                for n in self.names:
                    t = rng.expovariate(BUMPS_PER_H / 3600.0)
                    while t < T:
                        a = rng.uniform(0, 2 * math.pi)
                        d = rng.uniform(*BUMP_CM)
                        self.bumps[n].append([t, d * math.cos(a), d * math.sin(a), False])
                        t += rng.expovariate(BUMPS_PER_H / 3600.0)
            self.hung_at = None
            if "hung" in self.faults:
                n = rng.choice(self.names)
                t0 = rng.uniform(0.2, 0.6) * T
                L = rng.uniform(60.0, 180.0)
                self.slip[n].append((t0, t0 + L, 0.0))
                self.hung_at = (n, round(t0, 1), round(L, 1))
            # dark patch: a 30 x 30 cm square somewhere on the board (sightings 30 % there)
            B = self.board
            self.dark = []
            if "dark" in self.faults:
                x0, y0 = rng.uniform(B[0] + 15, B[2] - 45), rng.uniform(B[1] + 15, B[3] - 45)
                self.dark = [(x0, y0, x0 + 30.0, y0 + 30.0)]
            self.restart = None
            if "restart" in self.faults:
                self.restart = dict(name=rng.choice(self.names), t=rng.uniform(0.3, 0.7) * T,
                                    dur=rng.uniform(8.0, 15.0), done=False)
            self.wrong = None
            if "wrong_names" in self.faults:
                t0 = rng.uniform(0.05, 0.3) * T
                self.wrong = (t0, t0 + min(600.0, 0.15 * T))
            self.missing = None
            if "missing_cam" in self.faults:
                t0 = rng.uniform(0.5, 0.8) * T
                self.missing = (t0, t0 + min(600.0, 0.15 * T))
            # the side camera's own offset from the overhead (MEASURED on 11 Nov: the bridge learned (-3.44, 0.21) cm and
            # corrected it; what is left is CHOSEN here as 1 cm in a random direction)
            a = rng.uniform(0, 2 * math.pi)
            self.side_bias = (math.cos(a), math.sin(a))
            # hooks: pictures, sightings, deliveries, the network
            src = self.src
            src._frame0 = src.frame
            src.frame = self._frame
            src._deliver0 = src._deliver
            src._deliver = self._deliver
            src.layer._feed0 = src.layer.feed
            src.layer.feed = self._feed
            self.down._send0, self.down.send = self.down.send, self._down_send
            self.upn._send0, self.upn.send = self.upn.send, self._up_send
            self.upn._due0, self.upn.due = self.upn.due, self._up_due

        def _stalled(self, t=None):
            t = self.t if t is None else t
            k = bisect.bisect_right(self._stall_t0, t) - 1
            return k >= 0 and t < self.stalls[k][1]

        def _wifi_out(self, n, t):
            return any(a <= t < b for a, b in self.wifi.get(n, ()))

        # -- the network ------------------------------------------------------------------------------------------
        def _down_send(self, t, fn, *args):
            if args and self._wifi_out(args[0], t):
                return
            self.down._send0(t, fn, *args)

        def _up_send(self, t, fn, *args):
            if args and self._wifi_out(args[0], t):
                return
            self.upn._send0(t, fn, *args)

        def _up_due(self, t):
            if self._stalled(t):
                return []                   # the controller is stalled: robot messages wait in the socket
            return self.upn._due0(t)

        def _deliver(self):
            if self._stalled():
                return                      # fixes wait too, and arrive together when it runs again
            self.src._deliver0()

        # -- the camera -------------------------------------------------------------------------------------------
        def _fps_now(self):
            k = int(self.t // 10.0)
            v = self._fps.get(k)
            if v is None:
                v = self._fps[k] = max(0.5, qsample(self.prof["pictures_s"], self.rrng))
            return v

        def _replay_pic(self):
            """The replayed timeline's next picture: (time to the one after it, this one's delay)."""
            R_ = self._replay
            n = len(R_["pt"])
            i = R_["i"] % n
            j = (i + 1) % n
            gap = R_["pt"][j] - R_["pt"][i] if j > i else 0.5
            R_["i"] = i + 1
            return max(0.04, min(gap, 5.0)), R_["pd"][i]

        def _frame(self):
            src = self.src
            before = src.next_t
            src._frame0()
            if src.next_t != before:            # a picture was taken now: the next one at the measured rate
                self.m_pics.append(self.t)
                if self._replay is not None:
                    gap, d = self._replay_pic()
                    src.next_t = self.t + gap
                    d *= 2.0 if "latency2" in self.faults else 1.0
                else:
                    src.next_t = self.t + 1.0 / self._fps_now()
                    d = self._delay_draw()      # this picture's fixes arrive after the measured delay
                src.q = [(self.t + d, tf, fx) if tf == self.t else (due, tf, fx) for due, tf, fx in src.q]

        def _feed_cams(self, t, dets):
            """(e20) Several cameras, each sees every robot on its own (cams=[dict(name, share, sd, aniso=(ratio,
            axis_deg), cover=rect, dark=[rects], bump=(t0, deg), bias=(dx, dy))]); cam_swap=(t0, t1, a, b) swaps two
            cameras' names for a spell, each picture then read through the other's calibration (mirrored left-right)."""
            P, rng = self.prof, self.rrng
            keep = [d for d in dets if getattr(d, "src", "") == "tags" or d.name is not None and d.src != "lock"]
            alive = [b for b in self.bots.values() if not b.dead]
            merged = set()
            for i in range(len(alive)):
                for j in range(i + 1, len(alive)):
                    a, b = alive[i], alive[j]
                    if math.hypot(a.x - b.x, a.y - b.y) - FC.BODY < MERGE_GAP_CM:
                        merged.update((a.name, b.name))
            B = self.board
            cx, cy = 0.5 * (B[0] + B[2]), 0.5 * (B[1] + B[3])
            sw = self.cam_swap
            swapping = sw is not None and sw[0] <= t < sw[1]
            out = []
            for c in self.cams_spec:
                name = c["name"]
                for b in alive:
                    if self.cam_gap(b.name, t) or not self.in_cover(b.x, b.y) or b.name in merged:
                        continue
                    cv = c.get("cover")
                    if cv and not (cv[0] <= b.x <= cv[2] and cv[1] <= b.y <= cv[3]):
                        continue
                    p = float(c.get("share", 0.95))
                    dark = any(r[0] <= b.x <= r[2] and r[1] <= b.y <= r[3] for r in c.get("dark") or ())
                    if dark:
                        p *= 0.3
                    if rng.random() > p:
                        continue
                    sd = float(c.get("sd", 0.7))
                    ex, ey = rng.gauss(0, sd), rng.gauss(0, sd)
                    an = c.get("aniso")
                    if an:
                        a_ = math.radians(an[1])
                        along = ex * an[0]
                        ex, ey = along * math.cos(a_) - ey * math.sin(a_), along * math.sin(a_) + ey * math.cos(a_)
                    x, y = b.x + ex, b.y + ey
                    if c.get("bias"):
                        x, y = x + c["bias"][0], y + c["bias"][1]
                    bm = c.get("bump")
                    if bm and t >= bm[0]:
                        a_ = math.radians(bm[1])
                        x, y = (cx + (x - cx) * math.cos(a_) - (y - cy) * math.sin(a_),
                                cy + (x - cx) * math.sin(a_) + (y - cy) * math.cos(a_))
                    if self.cam_fault or self.cam_correct is not None:
                        x, y = self.cam_map(x, y)
                    label = name
                    if swapping and name in (sw[2], sw[3]):
                        # each picture through the OTHER camera's calibration: two different transforms (a's mirrored
                        # left-right, b's top-bottom), so two swapped cameras never agree with each other by accident
                        label = sw[3] if name == sw[2] else sw[2]
                        if name == sw[2]:
                            x = B[0] + B[2] - x
                        else:
                            y = B[1] + B[3] - y
                    out.append(TL.Det("motion", t, x, y, cam=label, light="dark" if dark else None))
                    self.m_seen.setdefault(b.name, []).append(t)
                    self.m_lastdet[b.name] = (t, x, y)
            return self.src.layer._feed0(t, keep + out)

        def _feed(self, t, dets):
            """The bridge's sightings instead of fastsim_sources' per-source ones: one unnamed sighting a robot a
            picture, as the home bridge sends them (freeroam.det_of: "both" / "side" -> motion)."""
            if getattr(self, "cams_spec", None):
                return self._feed_cams(t, dets)
            P, rng = self.prof, self.rrng
            out = []
            keep = [d for d in dets if getattr(d, "src", "") == "tags" or d.name is not None and d.src != "lock"]
            alive = [b for b in self.bots.values() if not b.dead]
            merged = set()
            for i in range(len(alive)):
                for j in range(i + 1, len(alive)):
                    a, b = alive[i], alive[j]
                    if math.hypot(a.x - b.x, a.y - b.y) - FC.BODY < MERGE_GAP_CM:
                        merged.update((a.name, b.name))
            share = dict(P.get("src_share") or {"both": 1.0})
            sd = P.get("standing_sd_cm") or {}
            miss = self.missing is not None and self.missing[0] <= t < self.missing[1]
            wrong = self.wrong is not None and self.wrong[0] <= t < self.wrong[1]
            B = self.board
            for b in alive:
                if self.cam_gap(b.name, t) or not self.in_cover(b.x, b.y) or b.name in merged:
                    continue
                p = (P.get("seen_pct") or {}).get(b.name)
                p = (sum((P.get("seen_pct") or {"x": 95.0}).values()) / max(1, len(P.get("seen_pct") or {"x": 1})))\
                    if p is None else p
                p /= 100.0
                if any(r[0] <= b.x <= r[2] and r[1] <= b.y <= r[3] for r in self.dark):
                    p *= 0.3
                if miss and b.y > B[1] + 0.75 * (B[3] - B[1]):
                    continue                     # the side camera alone does not see the far quarter
                if rng.random() > p:
                    continue
                if miss:
                    cam = "side"
                else:
                    u, cam = rng.random(), "both"
                    for k, v in share.items():
                        u -= v
                        if u <= 0:
                            cam = k
                            break
                s_ = float(sd.get(cam) or sd.get("both") or 0.7)
                x, y = b.x + rng.gauss(0, s_), b.y + rng.gauss(0, s_)
                if cam == "side":
                    x, y = x + self.side_bias[0], y + self.side_bias[1]
                if self.cam_fault or self.cam_correct is not None:
                    x, y = self.cam_map(x, y)
                if wrong and cam in ("side", "both"):
                    x = B[0] + B[2] - x          # the other camera's calibration: mirrored left to right
                dark = any(r[0] <= b.x <= r[2] and r[1] <= b.y <= r[3] for r in self.dark)
                d = TL.Det("motion", t, x, y, cam=cam, light="dark" if dark else None)    # (e20) its camera
                out.append(d)
                self.m_seen.setdefault(b.name, []).append(t)
                self.m_lastdet[b.name] = (t, x, y)
            return self.src.layer._feed0(t, keep + out)

        # -- every step -------------------------------------------------------------------------------------------
        def truth(self):
            if self._real:
                self._slips()
                self._bumps()
                self._restart()
                k = bisect.bisect_right(self._stall_t0, self.t + F.DT) - 1      # a stall due by the next step
                if k >= 0 and self.t + F.DT < self.stalls[k][1]:
                    self.frozen_until = max(self.frozen_until, self.stalls[k][1])
            super().truth()
            self._measure_step()

        def _slips(self):
            for n, b in self.bots.items():
                prev = self._prev.get(n)
                self._prev[n] = (b.x, b.y, b.th)
                if prev is None or b.dead:
                    continue
                for a, e, f in self.slip[n]:
                    if a <= self.t < e:
                        b.x, b.y = prev[0] + f * (b.x - prev[0]), prev[1] + f * (b.y - prev[1])
                        b.th = F.wrap180(prev[2] + f * F.wrap180(b.th - prev[2]))
                        self._prev[n] = (b.x, b.y, b.th)
                        self.m_slip_s[n] = self.m_slip_s.get(n, 0.0) + F.DT
                        break

        def _bumps(self):
            """(e21, 5) A due push moves the body (truth), not its wheels or its own pose; counted."""
            for n, b in self.bots.items():
                for q in getattr(self, "bumps", {}).get(n, ()):
                    if not q[3] and q[0] <= self.t:
                        q[3] = True
                        if b.dead:
                            continue
                        b.x += q[1]
                        b.y += q[2]
                        if n in self._prev:
                            self._prev[n] = (b.x, b.y, b.th)
                        self.stats["bumps"] = self.stats.get("bumps", 0) + 1

        def _restart(self):
            R = self.restart
            if not R or R["done"] or self.t < R["t"]:
                return
            b = self.bots[R["name"]]
            if not getattr(b, "_rebooting", False):
                b._rebooting = True
                b.mute = True
                b.q, b.cur, b.v, b.state, b.coast = [], None, 0.0, "idle", None
                self.ctrl.event(self.ctrl.R[b.name], "restarting (sim)", "silent for %.0f s, queue lost" % R["dur"])
            elif self.t >= R["t"] + R["dur"]:
                b.mute, b._rebooting, R["done"] = False, False, True
                # the runner's on_boot: safety set again, compass check, follow again; its own pose from the camera's
                # spot, its heading from a fresh gyro (anywhere)
                b.ox, b.oy = b.x + self.rrng.gauss(0, 0.7), b.y + self.rrng.gauss(0, 0.7)
                b.oth = F.wrap180(b.th + self.rrng.uniform(-180, 180))
                b.hist = []
                r = self.ctrl.R[b.name]
                if r.move is not None:
                    self.ctrl.end_move(r)
                if hasattr(self.ctrl, "on_restart"):
                    # (e19) as the runner's on_boot / follow restart now do: its pose re-established, a heading check
                    self.ctrl.on_restart(r)
                    self.ctrl.follow_again(r)
                self.ctrl.event(r, "restarted (sim)", "own heading %.0f deg off" % F.wrap180(b.oth - b.th))
                R["hdg_off"] = round(F.wrap180(b.oth - b.th), 1)

        # -- measures (the same definitions as tools/analysis/realism_measure.py) ---------------------------------
        def _init_measures(self):
            self._prev = {}
            self.m_pics, self.m_seen, self.m_named, self.m_delay = [], {}, {}, []
            self.m_lastdet, self.m_quiet, self.m_owncam, self.m_slip_s = {}, {n: [] for n in self.names}, [], {}
            self.m_still = {n: 0.0 for n in self.names}
            self._mprev = {}
            self._still_hung = {}
            self.m_close = {}
            self.m_stands = []
            self.m_blind = {n: dict(t_fix=0.0, d=0.0, last=None) for n in self.names}
            self.m_breach = []
            self.m_blind_legs = []
            self._mt = 0.0
            self._stall_t0 = [a for a, _ in getattr(self, "stalls", [])]
            c = self.ctrl
            c._on_fix0 = c.on_fix

            def on_fix(name, t, x, y, th=None, **kw):
                arrival = kw.get("arrival")
                if kw.get("how") is None and arrival is not None:
                    self.m_delay.append(arrival - t)
                    self.m_named.setdefault(name, []).append(t - self.cam_clock)
                    self.m_blind[name]["t_fix"] = self.t
                    self.m_blind[name]["d"] = 0.0
                return c._on_fix0(name, t, x, y, th, **kw)
            c.on_fix = on_fix
            if not getattr(self, "_real", False):
                # the easy profile: its pictures are fastsim_sources' own
                if self.src is not None:
                    src = self.src
                    src._frame0 = src.frame

                    def frame():
                        before = src.next_t
                        src._frame0()
                        if src.next_t != before:
                            self.m_pics.append(self.t)
                            for b in self.bots.values():
                                if not b.dead and not self.cam_gap(b.name, self.t):
                                    self.m_seen.setdefault(b.name, []).append(self.t)
                                    self.m_lastdet[b.name] = (self.t, b.x, b.y)
                    src.frame = frame

        def _measure_step(self):
            t = self.t
            if t - self._mt < 0.1 - 1e-9:
                return
            self._mt = t
            sec = int(t * 10) % 10 == 0
            al = [b for b in self.bots.values() if not b.dead]
            k10 = int(t // 10)
            for i in range(len(al)):
                for j in range(i + 1, len(al)):
                    dd = math.hypot(al[i].x - al[j].x, al[i].y - al[j].y)
                    self.m_close[k10] = min(self.m_close.get(k10, 999.0), dd)
            for n, b in self.bots.items():
                r = self.ctrl.R[n]
                alive = not b.dead and r.mode not in ("out", "gone", "start")
                if not b.dead:
                    self.m_quiet[n].append((t, t - b.last_rx))
                # never still: a stand of an alive robot = its body neither moving (0.3 cm/s) nor turning (2 deg/s)
                pv = self._mprev.get(n)
                self._mprev[n] = (b.x, b.y, b.th)
                moved = pv is None or math.hypot(b.x - pv[0], b.y - pv[1]) >= 0.03 or \
                    abs(F.wrap180(b.th - pv[2])) >= 0.2
                if alive and not moved:
                    self.m_still[n] += 0.1
                    if getattr(b, "hung", False) or any(a_ <= t < e_ and f_ < 0.05 for a_, e_, f_ in
                                                        getattr(self, "slip", {}).get(n, ())):
                        self._still_hung[n] = True      # hung at an edge, or wheels spinning: the body cannot move
                elif self.m_still[n] > 0:
                    self._stand_end(n, r, t)
                # blind drives: distance driven since the last named fix; breach = the run's position further from
                # the truth than its own stated doubt (ctrl.unc) while blind
                bl = self.m_blind[n]
                if bl["last"] is not None and alive:
                    bl["d"] += math.hypot(b.x - bl["last"][0], b.y - bl["last"][1])
                bl["last"] = (b.x, b.y)
                if alive and sec and t > 10.0 and t - bl["t_fix"] > 1.0 and bl["d"] > 2.0:
                    p = self.ctrl.pos(r)
                    if p is not None:
                        e = math.hypot(p[0] - b.x, p[1] - b.y)
                        u = self.ctrl.unc(r)
                        self.m_blind_legs.append((round(bl["d"], 1), round(e, 2), round(u, 2)))
                        if e > u:
                            self.m_breach.append((round(t, 1), n, round(bl["d"], 1), round(e, 1), round(u, 1)))
                if sec and alive and n in self.m_lastdet and t - self.m_lastdet[n][0] <= 0.3:
                    ld = self.m_lastdet[n]
                    self.m_owncam.append(math.hypot(ld[1] - b.ox, ld[2] - b.oy))

        def _stand_end(self, n, r, t):
            if self.m_still[n] >= NEVER_STILL_S:
                self.m_stands.append((round(t - self.m_still[n], 1), n, round(self.m_still[n], 1), r.mode,
                                      (r.held or {}).get("kind") if getattr(r, "held", None) else None,
                                      bool(self._still_hung.get(n))))
            self.m_still[n] = 0.0
            self._still_hung[n] = False

        def _close_rates(self, h):
            """10 s windows whose closest pair of middles came under 15 / 13.44 (the 6.72 cm circles touch) / 11 cm,
            per hour (realism_measure.close_rates on the real run's bridge sightings)."""
            v = list(self.m_close.values())
            return {k: round(sum(1 for x in v if x < lim) / h, 1) for k, lim in (("under_15", 15.0),
                                                                                  ("under_13_44", 13.44),
                                                                                  ("under_11", 11.0))}

        def measure_sim(self):
            import realism_measure as RM
            h = max(1e-6, self.t / 3600.0)
            names = self.names
            rh = h * len(names)
            Q = RM.quant
            pics = sorted(self.m_pics)
            ps = RM.picture_spells(pics, self.m_seen, self.m_named, names)
            qs, st_all, shared = RM.quiet_spells({n: list(v) for n, v in self.m_quiet.items()})
            kept = getattr(self, "kept", {})
            ev = kept.get("events", [])

            def rate(kind):
                return round(sum(1 for e in ev if e["kind"] == kind) / rh, 2)
            import re
            he = []
            for e in ev:
                mm = re.search(r"its own heading (-?[\d.]+) deg, the camera sees it go (-?[\d.]+)", e.get("text", ""))
                if mm:
                    he.append(abs((float(mm.group(1)) - float(mm.group(2)) + 180.0) % 360.0 - 180.0))
                mm = re.search(r"drove (-?[\d.]+) deg off its own heading", e.get("text", ""))
                if e["kind"] == "heading re-set" and mm:
                    he.append(abs(float(mm.group(1))))
            pw = {}
            for p in pics:
                pw[int(p // 10)] = pw.get(int(p // 10), 0) + 1
            lost = kept.get("lost", [])
            return dict(run="sim-%s-seed%d" % (self.prof["name"], self.seed), hours=round(h, 3), robots=names,
                        pictures_s=Q([v / 10.0 for k, v in pw.items() if k < int(self.t // 10)]),
                        fix_delay_s=Q(self.m_delay), quiet_s=Q(qs),
                        stalls=dict(per_robot_h=round(len(st_all) / rh, 2), length_s=Q([q for *_, q in st_all]),
                                    shared_pct=round(100.0 * shared / max(1, len(st_all)), 1),
                                    over_1_5s_per_robot_h=round(sum(1 for *_, q in st_all if q >= 1.5) / rh, 2)),
                        watchdog_per_h=round(self.stats["watchdog_stops"] / h, 1),
                        unnamed=dict(per_robot_h=round(len(ps["unnamed"]) / rh, 2), length_s=Q(ps["unnamed"]),
                                     over_5s_per_robot_h=round(sum(1 for u in ps["unnamed"] if u > 5) / rh, 3),
                                     longest_s=round(max(ps["unnamed"]), 1) if ps["unnamed"] else 0.0),
                        cam_gaps=dict(per_robot_h=round(len(ps["gaps"]) / rh, 2), length_s=Q(ps["gaps"])),
                        seen_pct=ps["seen_pct"],
                        own_cam_gap_cm=Q(self.m_owncam),
                        close=self._close_rates(h),
                        heading_err_deg=dict(per_robot_h=round(len(he) / rh, 2), size=Q(he),
                                             over_20_per_robot_h=round(sum(1 for x in he if x > 20) / rh, 3)),
                        did_not_move=dict(per_robot_h=rate("did not move")),
                        needs_hand=dict(per_robot_h=rate("NEEDS A HAND")), rejoined=dict(per_robot_h=rate("REJOINED")),
                        front_stops=dict(per_robot_h=rate("front stop")),
                        lost=dict(per_robot_h=round(len(lost) / rh, 2),
                                  standing_per_robot_h=round(sum(1 for r in lost if float(r.get("blind_cm") or 99) < 1)
                                                             / rh, 2),
                                  length_s=Q([float(r["dur_s"]) for r in lost])))

        def breakage(self, summ=None):
            """What broke in this run (the stress batch's columns)."""
            s = summ or {}
            for n in self.names:                  # a stand still going on at the end counts too
                if self.m_still[n] > 0:
                    self._stand_end(n, self.ctrl.R[n], self.t)
            kept = getattr(self, "kept", {})
            ev = kept.get("events", [])
            hands = [e for e in ev if e["kind"] == "NEEDS A HAND"]
            src = self.src.summary() if self.src is not None else {}
            legs = self.m_blind_legs
            return dict(contacts=self.stats["contacts"], contact_s=round(self.stats["contact_s"], 2),
                        min_gap_cm=round(self.stats["min_gap"], 2), min_gap_at=self.stats.get("min_gap_at"),
                        falls=self.stats.get("falls", 0), off_board=self.stats["off_board"],
                        hangs=self.stats.get("hangs", 0), hung_s=round(self.stats.get("hung_s", 0.0), 1),
                        swaps=src.get("swaps", 0), wrong_fixes=src.get("wrong_fixes"),
                        needs_hand=len(hands), rejoined=sum(1 for e in ev if e["kind"] == "REJOINED"),
                        holds_end={n: (r.held or {}).get("kind") for n, r in self.ctrl.R.items()
                                   if getattr(r, "held", None)},
                        stands_over=len(self.m_stands), stands=self.m_stands[:30],
                        longest_stand_s=max([q[2] for q in self.m_stands] + [0.0]),
                        stands_free=sum(1 for q in self.m_stands if not q[5]),
                        longest_free_stand_s=max([q[2] for q in self.m_stands if not q[5]] + [0.0]),
                        blind_samples=len(legs), blind_breaches=len(self.m_breach), breaches=self.m_breach[:30],
                        blind_err_p95=RMq(legs, 1, 0.95), blind_d_p95=RMq(legs, 0, 0.95),
                        watchdog_stops=self.stats["watchdog_stops"],
                        out={n: r.out_why for n, r in self.ctrl.R.items() if r.mode in ("out", "gone")},
                        moving_pct=s.get("moving_pct"), hung_at=getattr(self, "hung_at", None),
                        restart=getattr(self, "restart", None), faults=sorted(getattr(self, "faults", {}) or ()),
                        counts={k: self.ctrl.counts.get(k) for k in (
                            "camera_corrections", "need_hand", "creeps", "late_slowed", "late_waited", "reseeds",
                            "rejoined", "freed_itself", "corrections_revoked", "hd_confirm_crawls")
                            if k in self.ctrl.counts})

    return HardSim


def RMq(rows, i, q):
    v = sorted(r[i] for r in rows)
    return round(v[min(len(v) - 1, int(q * len(v)))], 2) if v else None


def main(argv=None):
    """python3 tools/fleet/fastsim_realism.py build MEASURED.json RUN_NAME OUT_PROFILE.json"""
    a = list(sys.argv[1:] if argv is None else argv)
    if len(a) == 4 and a[0] == "build":
        runs = json.load(open(a[1]))["runs"]
        m = [r for r in runs if r["run"] == a[2]][0]
        p = build_profile(m, name="measured-" + a[2])
        with open(a[3], "w") as f:
            json.dump(p, f, indent=1)
        print("profile", a[3])
        return
    print(main.__doc__)


if __name__ == "__main__":
    main()
