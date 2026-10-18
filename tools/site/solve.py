"""Joint solve of the site map: where every landmark and open edge is, with how
sure, from everything the survey measured at once (a bundle adjustment / factor
graph, solved by robust least squares).

Unknowns
  per landmark   move dx, dy (cm) and turn (deg) about its centre, from where the
                 user's clicks (or the map) put it
  per robot      each front-sensor zone's real direction (offset from 22, 11, 0,
                 -11, -22 deg), range bias (cm) and range scale
  per station    the robot's pose (x, y, heading) while it stood still
  per open edge  offset along its outward normal (cm) and turn (deg)
  edge guard     how far past the edge the line sensors are when the guard trips
                 (one number, cm)

Measurements (each a residual divided by its standard deviation)
  camera         each station's position (median of its sightings: sd = camera
                 noise / sqrt(sightings)) and heading if the camera gives one
  wheels + gyro  the move between consecutive stations of a run (sd 0.3 cm + 3%
                 of the path; heading 1 deg + 2% of the turn)
  front sensor   each zone's median reading at a station, against the distance
                 along that zone's ray to the face it hits (sd: reading noise /
                 sqrt(frames/2), plus 0.15 cm that does not average away)
  edge guard     a trip: the leading line sensor is k cm past the edge (sd 0.15 cm)
  user           clicks: each landmark's move and turn (from the click sd per end);
                 edges from the map (1 cm); tape measurements, once taken
  calibration    per robot: zone directions, bias and scale from the per-robot
                 calibration (tof_calibration.json) or the bench rows (E0c C9),
                 else nominal with broad priors (5 deg, 1 cm, 3%)

The camera frame's own error (the floor calibration's) is common to every
station near a landmark, so it does not average away: it is added to every
point's uncertainty at the end (Estimate.cam_sys_cm), not inside the solve.

Uncertainty: the inverse of J^T J at the solution (Gauss-Newton), scaled up by
the fit's reduced chi-square when that is above 1.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
for _p in (HERE, TOOLS, os.path.join(TOOLS, "fusion")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import engine as ENG  # noqa: E402
import landmarks as LM  # noqa: E402
import sitemap as SM  # noqa: E402
import observe as OB  # noqa: E402

try:
    from scipy.optimize import least_squares
    from scipy.sparse import lil_matrix
except ImportError:   # pragma: no cover
    least_squares = None

D2R = math.pi / 180.0
LINE_FWD_CM, LINE_LAT_CM = 3.5, 1.2

DEFAULTS = dict(
    cam_sigma_cm=0.3,          # one camera sighting's position noise
    cam_sigma_deg=3.0,         # one sighting's heading noise (the detector's heading is rough)
    cam_floor_cm=0.03,         # a station's camera position can't be better than this
    cam_sys_cm=0.15,           # the floor calibration's own error (shared, added at the end)
    use_cam_heading=True,
    tof_noise_cm=0.5,          # one front-sensor reading's noise: this + rel x distance
    tof_noise_rel=0.01,
    tof_sys_cm=0.15,           # per station and zone, what does not average away
    tof_zone_width_deg=4.0,    # zone footprint (start value; fitted, 0..11 deg, prior sd width_prior_deg)
    width_prior_deg=5.0,
    corners=True, corner_sd_deg=0.2,
    tof_max_incidence_deg=30.0,
    odo_cm=0.3, odo_rel=0.03, odo_deg=0.3, odo_turn_rel=0.02,
    prior_zone_deg=5.0, prior_bias_cm=1.0, prior_scale=0.03,
    edge_prior_cm=1.0, edge_prior_deg=1.0, edge_k0_cm=0.2, edge_k_sd_cm=0.5, edge_trip_sd_cm=0.15,
    tape_sd_cm=0.1,
    gate_first_cm=3.0, gate_sd=4.0,
    iterations=3, f_scale=2.0, max_nfev=80,
)


class Problem:
    """Everything the solver needs: landmarks, edges, stations with readings, links, priors."""

    def __init__(self, landmarks, edges, stations, links, robots, opts=None, robot_priors=None, tape=None):
        self.o = dict(DEFAULTS, **(opts or {}))
        self.L = list(landmarks)
        self.E = [e for e in edges if e.open]
        self.all_edges = list(edges)
        self.S = list(stations)
        self.links = list(links)
        self.robots = list(robots)
        self.rp = robot_priors or {}
        self.tape = list(tape or [])          # [(landmark, anchor, (x, y))] measured by tape, used as data
        self.cfg = ENG.config(tof_zone_width_deg=self.o["tof_zone_width_deg"],
                              tof_max_incidence_deg=self.o["tof_max_incidence_deg"])
        self.fwd = self.cfg["tof_forward_cm"] - self.cfg["tof_recess_cm"]
        self.nom = np.array(self.cfg["tof_zone_deg"], float)
        # faces, with their landmark (group) index
        self.faces, self.fg = [], []
        for gi, l in enumerate(self.L):
            for f in l.faces:
                self.faces.append(f)
                self.fg.append(gi)
        self.fg = np.array(self.fg, int)
        self.C = np.array([l.center for l in self.L])
        self.A = np.array([f.a for f in self.faces])
        self.N = np.array([f.n for f in self.faces])
        self.ri = {r: i for i, r in enumerate(self.robots)}
        self.sr = np.array([self.ri[s.robot] for s in self.S], int)
        nG, nR, nS, nE = len(self.L), len(self.robots), len(self.S), len(self.E)
        self.oG, self.oR = 0, 4 * nG
        # stretch: each face's landmark axis and length (boards only)
        self.axis = np.array([self.L[gi].axis for gi in self.fg]) if len(self.faces) else np.zeros((0, 2))
        self.Llen = np.array([max(self.L[gi].length, 1e-9) for gi in self.fg]) if len(self.faces) else np.zeros(0)
        self.isboard = np.array([self.L[gi].length > 0 for gi in self.fg], bool) if len(self.faces) else np.zeros(0, bool)
        self.B = np.array([f.b for f in self.faces])
        self.oS = self.oR + 7 * nR
        self.oE = self.oS + 3 * nS
        self.oK = self.oE + 2 * nE
        self.oW = self.oK + 1
        self.n = self.oW + 1
        self.corners = []        # corner-bearing observations (see find_corners)
        self.obs = None
        self.trips = []          # (station index, edge index)
        self._edge_assoc()
        self.gyro_sign = self._gyro_sign()

    # ------------------------------------------------------------ parameter vector
    def x0(self):
        p = np.zeros(self.n)
        for i, r in enumerate(self.robots):
            pr = self.rp.get(r, {})
            p[self.oR + 7 * i:self.oR + 7 * i + 5] = pr.get("zone_offset_deg", [0.0] * 5)
            p[self.oR + 7 * i + 5] = pr.get("bias_cm", 0.0)
            p[self.oR + 7 * i + 6] = pr.get("scale", 1.0) - 1.0
        for i, s in enumerate(self.S):
            th = s.cam_th if s.cam_th is not None else self._heading_guess(s)
            p[self.oS + 3 * i:self.oS + 3 * i + 3] = [s.cam[0], s.cam[1], th * D2R]
        p[self.oK] = self.o["edge_k0_cm"]
        p[self.oW] = self.o["tof_zone_width_deg"]
        return p

    def bounds(self):
        lo = np.full(self.n, -np.inf)
        hi = np.full(self.n, np.inf)
        lo[self.oW], hi[self.oW] = 0.0, 11.0
        return lo, hi

    def _heading_guess(self, s):
        return getattr(s, "heading_guess", 0.0)

    def unpack(self, p):
        nG, nR, nS, nE = len(self.L), len(self.robots), len(self.S), len(self.E)
        g = p[self.oG:self.oR].reshape(nG, 4)
        r = p[self.oR:self.oS].reshape(nR, 7)
        X = p[self.oS:self.oE].reshape(nS, 3)
        e = p[self.oE:self.oK].reshape(nE, 2)
        return g, r, X, e, p[self.oK]

    def width(self, p):
        return float(p[self.oW])

    # ------------------------------------------------------------ geometry
    def _move(self, P, g):
        """Points P (one per face) of each face's landmark, moved by its g (dx, dy, turn, stretch)."""
        cen = self.C[self.fg]
        rel = P - cen
        st = np.where(self.isboard, g[self.fg, 3] / self.Llen, 0.0)
        along = np.sum(rel * self.axis, 1)
        rel = rel + (along * st)[:, None] * self.axis
        phi = g[self.fg, 2] * D2R
        c, s = np.cos(phi), np.sin(phi)
        return cen + np.c_[c * rel[:, 0] - s * rel[:, 1], s * rel[:, 0] + c * rel[:, 1]] + g[self.fg, :2]

    def faces_at(self, g):
        """Moved faces: (start points a, normals n, lengths)."""
        a = self._move(self.A, g)
        b = self._move(self.B, g)
        v = b - a
        L = np.hypot(v[:, 0], v[:, 1])
        u = v / np.maximum(L, 1e-9)[:, None]
        n = np.c_[-u[:, 1], u[:, 0]]
        self._flen = L
        return a, n

    def moved_faces(self, g):
        out = []
        for gi, l in enumerate(self.L):
            out += l.moved_faces(g[gi])
        return out

    def predict_tof(self, p, obs=None):
        obs = self.obs if obs is None else obs
        g, r, X, _, _ = self.unpack(p)
        a_f, n_f = self.faces_at(g)
        si, zi, fi = obs["s"], obs["z"], obs["f"]
        rb = self.sr[si]
        th = X[si, 2]
        px, py = X[si, 0] + self.fwd * np.cos(th), X[si, 1] + self.fwd * np.sin(th)
        a, n = a_f[fi], n_f[fi]
        ang = (self.nom[zi] + r[rb, zi]) * D2R
        w = self.width(p)
        if w > 1e-6:
            side = (a[:, 0] - px) * n[:, 0] + (a[:, 1] - py) * n[:, 1]
            m = n * np.sign(side)[:, None]
            rel = np.arctan2(m[:, 1], m[:, 0]) - th - ang
            rel = (rel + np.pi) % (2 * np.pi) - np.pi
            ang = ang + np.clip(rel, -w * D2R / 2, w * D2R / 2)
        ux, uy = np.cos(th + ang), np.sin(th + ang)
        nu = n[:, 0] * ux + n[:, 1] * uy
        nu = np.where(np.abs(nu) < 1e-6, 1e-6, nu)
        dist = ((a[:, 0] - px) * n[:, 0] + (a[:, 1] - py) * n[:, 1]) / nu
        return dist * (1 + r[rb, 6]) + r[rb, 5]

    # ------------------------------------------------------------ associations
    def associate(self, p, first=False):
        """Tie each station's zone readings to the face its ray hits (the fusion's own rules), keeping only
        readings near the prediction."""
        g, r, X, _, _ = self.unpack(p)
        faces = self.moved_faces(g)
        self.cfg["tof_zone_width_deg"] = self.width(p)
        Pm = np.diag([0.04, 0.04, (0.5 * D2R) ** 2])
        rows = []
        for si, st in enumerate(self.S):
            rb = self.sr[si]
            for zi, zz in enumerate(st.zones):
                if zz is None:
                    continue
                ang = (self.nom[zi] + r[rb, zi]) * D2R
                why, info = LM.predict_zone(X[si], Pm, ang, faces, self.cfg)
                if why:
                    continue
                h = info["h"] * (1 + r[rb, 6]) + r[rb, 5]
                sd = self.tof_sd(zz[0], zz[1])
                gate = self.o["gate_first_cm"] if first else max(self.o["gate_sd"] * sd, 0.6)
                if abs(zz[0] - h) > max(gate, 0.1 * h if first else 0.0):
                    continue
                rows.append((si, zi, info["face"], zz[0], sd))
        if not rows:
            self.obs = dict(s=np.zeros(0, int), z=np.zeros(0, int), f=np.zeros(0, int), v=np.zeros(0), sd=np.zeros(0))
            return 0
        c = list(zip(*rows))
        self.obs = dict(s=np.array(c[0], int), z=np.array(c[1], int), f=np.array(c[2], int),
                        v=np.array(c[3], float), sd=np.array(c[4], float))
        self.find_corners(p, gate=None if first else self.o["gate_sd"])
        return len(rows)

    # ------------------------------------------------------------ corner bearings
    def find_corners(self, p, gate=None):
        """Corner bearings. At a spot where the robot turns in small steps (still between them), a zone's
        reading goes from a landmark's face ("on": the distance to that face's line) to past it ("off": no
        reading in range, or further than the landmark's whole depth behind that line), maybe with readings
        in between (an end face seen at an angle). Between the last "on" and the first "off" heading, the
        landmark's outline corner on that side crossed the zone's edge ray. Stored as (station on, station
        off, zone, face index)."""
        g, r, X, _, _ = self.unpack(p)
        a_f, n_f = self.faces_at(g)
        faces = self.moved_faces(g)
        self.corners = []
        if not self.o.get("corners", True):
            return
        # sweeps: consecutive stations of one run at the same spot
        sweeps, cur = [], [0] if self.S else []
        for i in range(1, len(self.S)):
            A, B = self.S[cur[0]], self.S[i]
            if B.run == A.run and math.dist(A.cam, B.cam) < 1.5:
                cur.append(i)
            else:
                sweeps.append(cur)
                cur = [i]
        if cur:
            sweeps.append(cur)
        wr = 11.0 * D2R                      # the widest a zone could be: what its slice might touch
        depth = {}
        for fi, f in enumerate(self.faces):
            gi = self.fg[fi]
            pts = np.array([q for k, ff in enumerate(self.faces) if self.fg[k] == gi for q in (ff.a, ff.b)])
            depth[fi] = float(np.max(np.abs((pts - f.a) @ f.n)))
        for sw in sweeps:
            if len(sw) < 3:
                continue
            order = sorted(sw, key=lambda k: _wrap(X[k, 2] - X[sw[0], 2]))
            rb = self.sr[sw[0]]
            for zi in range(5):
                for fi, f in enumerate(self.faces):
                    if f.length < 15.0:
                        continue
                    cls = []
                    for si in order:
                        zz = self.S[si].zones[zi]
                        th = X[si, 2]
                        ps = X[si, :2] + self.fwd * np.array([math.cos(th), math.sin(th)])
                        ang = th + (self.nom[zi] + r[rb, zi]) * D2R
                        u = np.array([math.cos(ang), math.sin(ang)])
                        nu = float(np.dot(n_f[fi], u))
                        if abs(nu) < math.cos(50 * D2R):
                            cls.append(None)
                            continue
                        d = float(np.dot(a_f[fi] - ps, n_f[fi])) / nu
                        if d <= 2.0:
                            cls.append(None)
                            continue
                        h = d * (1 + r[rb, 6]) + r[rb, 5]
                        ua = np.array([n_f[fi][1], -n_f[fi][0]])          # along the face, a -> b (Face: n = (-u_y, u_x))
                        along = float(np.dot(ps + d * u - a_f[fi], ua))
                        if zz is None:
                            cls.append(("off", along))
                            continue
                        sd = self.tof_sd(zz[0], zz[1])
                        if abs(zz[0] - h) < max(1.0, 3 * sd):
                            cls.append(("on", along))
                        elif zz[0] > h + depth[fi] / max(abs(nu), 0.3) + 3.0:
                            cls.append(("off", along))
                        else:
                            cls.append(("mid", along))
                    if getattr(self, "debug_corners", None) is not None:
                        self.debug_corners.append((sw[0], zi, f.name, [None if c is None else (c[0], round(c[1], 1))
                                                                        for c in cls]))
                    # last "on" then first "off" (only "mid" between), in either turning direction
                    for seq in (list(range(len(order))), list(range(len(order) - 1, -1, -1))):
                        last_on = None
                        for k in seq:
                            c = cls[k]
                            if c is None:
                                last_on = None
                                continue
                            if c[0] == "on":
                                last_on = k
                            elif c[0] == "off" and last_on is not None:
                                son, soff = order[last_on], order[k]
                                gon, goff = cls[last_on][1], c[1]
                                L = float(self._flen[fi])
                                near_a = -4.0 < gon < 8.0 and goff < gon
                                near_b = L - 8.0 < gon < L + 4.0 and goff > gon
                                if (near_a or near_b) and abs(_wrap(X[soff, 2] - X[son, 2])) < 8.0 * D2R:
                                    th = X[son, 2]
                                    ps = X[son, :2] + self.fwd * np.array([math.cos(th), math.sin(th)])
                                    ang = th + (self.nom[zi] + r[rb, zi]) * D2R
                                    hits = [LM.nearest(ps, (math.cos(ang + k * wr), math.sin(ang + k * wr)), faces)
                                            for k in (-0.5, -0.25, 0.0, 0.25, 0.5)]
                                    hits = [hh for hh in hits if hh is not None]
                                    if hits and self.fg[min(hits)[1]] == self.fg[fi]:
                                        self.corners.append((son, soff, zi, fi))
                                last_on = None
                            elif c[0] == "off":
                                last_on = None
        self.corners = sorted(set(self.corners))
        if gate is not None and self.corners:
            r_ = self.corner_residuals(p)
            self.corners = [c for c, v in zip(self.corners, r_) if abs(v) <= gate]

    def corner_residuals(self, p):
        g, r, X, _, _ = self.unpack(p)
        a_f, n_f = self.faces_at(g)
        w = self.width(p) * D2R
        out = []
        verts = {}
        for son, soff, zi, fi in self.corners:
            gi = self.fg[fi]
            if gi not in verts:
                idx = np.flatnonzero(self.fg == gi)
                verts[gi] = np.r_[a_f[idx], self._move(self.B, g)[idx]]
            V = verts[gi]
            rb = self.sr[son]
            al, ps = [], []
            for si in (son, soff):
                th = X[si, 2]
                ps.append(X[si, :2] + self.fwd * np.array([math.cos(th), math.sin(th)]))
                al.append(th + (self.nom[zi] + r[rb, zi]) * D2R)
            dirn = 1.0 if _wrap(al[1] - al[0]) > 0 else -1.0
            # the outline's furthest vertex in the turning direction, as seen from the "on" pose
            b_on = np.arctan2(V[:, 1] - ps[0][1], V[:, 0] - ps[0][0])
            k = int(np.argmax(dirn * ((b_on - al[0] + np.pi) % (2 * np.pi) - np.pi)))
            q = []
            for j in (0, 1):
                beta = math.atan2(V[k, 1] - ps[j][1], V[k, 0] - ps[j][0])
                q.append(dirn * _wrap(beta - al[j]) + w / 2)      # the edge ray: half the zone width out
            step = abs(q[0] - q[1])
            sd = step / math.sqrt(12.0) + self.o["corner_sd_deg"] * D2R
            out.append(0.5 * (q[0] + q[1]) / sd)
        return np.array(out)

    def tof_sd(self, z, n):
        o = self.o
        return math.sqrt((o["tof_noise_cm"] + o["tof_noise_rel"] * z) ** 2 / max(1.0, n / 2.0) + o["tof_sys_cm"] ** 2)

    def _edge_assoc(self):
        """Which open edge each tripped station was facing (from the camera pose)."""
        self.trips = []
        for si, st in enumerate(self.S):
            if not st.edge or st.cam_th is None and not hasattr(st, "heading_guess"):
                continue
            th = (st.cam_th if st.cam_th is not None else st.heading_guess) * D2R
            u = np.array([math.cos(th), math.sin(th)])
            best = None
            for ei, e in enumerate(self.E):
                d = float(np.dot(e.mid - np.asarray(st.cam), e.n))
                if np.dot(u, e.n) > 0.85 and -5.0 < d < 25.0:
                    if best is None or d < best[0]:
                        best = (d, ei)
            if best:
                self.trips.append((si, best[1]))

    def _gyro_sign(self):
        num = sum(l[4] * self._odo_turn(l) for l in self.links)
        return -1.0 if num < 0 else 1.0

    def _odo_turn(self, l):
        a, b = self.S[l[0]], self.S[l[1]]
        return b.odo[2] - a.odo[2]

    # ------------------------------------------------------------ residuals
    def residuals(self, p):
        o = self.o
        g, r, X, e, k = self.unpack(p)
        out = []
        # front sensor
        if self.obs is not None and len(self.obs["v"]):
            out.append((self.obs["v"] - self.predict_tof(p)) / self.obs["sd"])
        # camera
        cam = []
        for si, st in enumerate(self.S):
            sd = max(o["cam_sigma_cm"] / math.sqrt(st.n_cam), o["cam_floor_cm"])
            cam += [(X[si, 0] - st.cam[0]) / sd, (X[si, 1] - st.cam[1]) / sd]
            if o["use_cam_heading"] and st.cam_th is not None:
                sdh = o["cam_sigma_deg"] / math.sqrt(st.n_cam) + 0.3
                cam.append(_wrap(X[si, 2] - st.cam_th * D2R) / (sdh * D2R))
        out.append(np.array(cam))
        # wheels + gyro between stations
        od = []
        for (i, j, along, left, dth, path, turned) in self.links:
            th = X[i, 2]
            dx, dy = X[j, 0] - X[i, 0], X[j, 1] - X[i, 1]
            pa = math.cos(th) * dx + math.sin(th) * dy
            pl = -math.sin(th) * dx + math.cos(th) * dy
            sdt = o["odo_cm"] + o["odo_rel"] * path
            sdr = (o["odo_deg"] + o["odo_turn_rel"] * abs(dth)) * D2R
            od += [(pa - along) / sdt, (pl - left) / sdt, _wrap(X[j, 2] - X[i, 2] - self.gyro_sign * dth * D2R) / sdr]
        if od:
            out.append(np.array(od))
        # edge trips
        tr = []
        for si, ei in self.trips:
            m, n = self.E[ei].line(e[ei])
            th = X[si, 2]
            u = np.array([math.cos(th), math.sin(th)])
            left = np.array([-u[1], u[0]])
            back = self.S[si].edge["back_cm"]
            q = X[si, :2] + (LINE_FWD_CM - back) * u
            lead = max(float(np.dot(q + lat * left - m, n)) for lat in (LINE_LAT_CM, 0.0, -LINE_LAT_CM))
            tr.append((lead - k) / o["edge_trip_sd_cm"])
        if tr:
            out.append(np.array(tr))
        # priors: landmarks (clicks), robots, edges, k
        pr = []
        for gi, l in enumerate(self.L):
            sp, sa, ss = l.prior_sigmas()
            pr += [g[gi, 0] / sp, g[gi, 1] / sp, g[gi, 2] / sa, g[gi, 3] / ss]
        for ri, rn in enumerate(self.robots):
            q = self.rp.get(rn, {})
            z0 = np.array(q.get("zone_offset_deg", [0.0] * 5))
            zs = np.array(q.get("zone_sd_deg", [o["prior_zone_deg"]] * 5))
            pr += list((r[ri, :5] - z0) / zs)
            pr.append((r[ri, 5] - q.get("bias_cm", 0.0)) / q.get("bias_sd_cm", o["prior_bias_cm"]))
            pr.append((r[ri, 6] - (q.get("scale", 1.0) - 1.0)) / q.get("scale_sd", o["prior_scale"]))
        for ei, ed in enumerate(self.E):
            pr += [e[ei, 0] / ed.prior_sigma_cm, e[ei, 1] / o["edge_prior_deg"]]
        pr.append((k - o["edge_k0_cm"]) / o["edge_k_sd_cm"])
        pr.append((self.width(p) - self.o["tof_zone_width_deg"]) / self.o["width_prior_deg"])
        out.append(np.array(pr))
        if self.corners:
            out.append(self.corner_residuals(p))
        # tape measurements used as data
        tp = []
        names = {l.name: gi for gi, l in enumerate(self.L)}
        for name, anchor, xy in self.tape:
            if name.startswith("edge "):
                side = name.split()[1]
                ei = [x.side for x in self.E].index(side)
                pt = self.E[ei].point(anchor, e[ei])
                _, n = self.E[ei].line(e[ei])
                tp.append(float(np.dot(np.asarray(xy) - pt, n)) / o["tape_sd_cm"])
                continue
            gi = names[name]
            pt = self.L[gi].move(self.L[gi].anchors[anchor], g[gi])
            tp += [(pt[0] - xy[0]) / o["tape_sd_cm"], (pt[1] - xy[1]) / o["tape_sd_cm"]]
        if tp:
            out.append(np.array(tp))
        return np.concatenate(out)

    def sparsity(self):
        """Which parameters each residual depends on (same order as residuals())."""
        rows = []
        o = self.o
        G3 = lambda gi: list(range(self.oG + 4 * gi, self.oG + 4 * gi + 4))
        R7 = lambda ri: list(range(self.oR + 7 * ri, self.oR + 7 * ri + 7))
        S3 = lambda si: list(range(self.oS + 3 * si, self.oS + 3 * si + 3))
        E2 = lambda ei: list(range(self.oE + 2 * ei, self.oE + 2 * ei + 2))
        if self.obs is not None:
            for si, fi in zip(self.obs["s"], self.obs["f"]):
                rows.append(G3(self.fg[fi]) + R7(self.sr[si]) + S3(si) + [self.oW])
        for si, st in enumerate(self.S):
            rows += [S3(si), S3(si)]
            if o["use_cam_heading"] and st.cam_th is not None:
                rows.append(S3(si))
        for (i, j, *_r) in self.links:
            rows += [S3(i) + S3(j)] * 3
        for si, ei in self.trips:
            rows.append(S3(si) + E2(ei) + [self.oK])
        for gi in range(len(self.L)):
            rows += [G3(gi)] * 4
        for ri in range(len(self.robots)):
            rows += [R7(ri)] * 7
        for ei in range(len(self.E)):
            rows += [E2(ei)] * 2
        rows.append([self.oK])
        rows.append([self.oW])
        for son, soff, zi, fi in self.corners:
            rows.append(G3(self.fg[fi]) + R7(self.sr[son]) + S3(son) + S3(soff) + [self.oW])
        names = {l.name: gi for gi, l in enumerate(self.L)}
        for name, anchor, xy in self.tape:
            if name.startswith("edge "):
                rows.append(E2([x.side for x in self.E].index(name.split()[1])))
            else:
                rows += [G3(names[name])] * 2
        M = lil_matrix((len(rows), self.n), dtype=int)
        for i, cols in enumerate(rows):
            for c in cols:
                M[i, c] = 1
        return M


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Result:
    def __init__(self, prob, p, cov, chi2, dof, n_tof, est, robots_out, noise):
        self.prob, self.p, self.cov, self.chi2, self.dof, self.n_tof = prob, p, cov, chi2, dof, n_tof
        self.est, self.robots_out, self.noise = est, robots_out, noise

    def summary(self):
        return dict(stations=len(self.prob.S), readings=int(self.n_tof), edge_trips=len(self.prob.trips),
                    corner_bearings=len(self.prob.corners), zone_width_deg=round(self.prob.width(self.p), 2),
                    chi2_per_dof=round(self.chi2 / max(self.dof, 1), 3))


def solve(prob, cam_sys_cm=None, verbose=False):
    """Solve; returns Result with an SM.Estimate (moves, covariances)."""
    if least_squares is None:
        raise RuntimeError("the site solver needs scipy: pip install scipy")
    o = prob.o
    p = prob.x0()
    sol = None
    for it in range(o["iterations"]):
        prob.associate(p, first=(it == 0))
        M = prob.sparsity()
        lo, hi = prob.bounds()
        p = np.clip(p, lo + 1e-6, hi - 1e-6)
        sol = least_squares(prob.residuals, p, jac_sparsity=M, loss="soft_l1", f_scale=o["f_scale"],
                            x_scale="jac", max_nfev=o["max_nfev"], method="trf", bounds=(lo, hi))
        p = sol.x
        if verbose:
            print(f"  pass {it + 1}: {len(prob.obs['v'])} readings tied to faces, {len(prob.corners)} corner "
                  f"bearings, zone width {prob.width(p):.1f} deg, cost {sol.cost:.1f}")
    J = sol.jac.toarray() if hasattr(sol.jac, "toarray") else np.asarray(sol.jac)
    res = prob.residuals(p)
    dof = max(len(res) - prob.n, 1)
    chi2 = 2.0 * float(sol.cost)          # robust (soft-L1) sum: about chi-square for the readings that fit
    H = J.T @ J
    try:
        cov = np.linalg.inv(H + np.eye(prob.n) * 1e-12)
    except np.linalg.LinAlgError:
        cov = np.linalg.pinv(H)
    infl = max(1.0, chi2 / dof)
    cov *= infl
    g, r, X, e, k = prob.unpack(p)
    cov_g = {}
    for gi, l in enumerate(prob.L):
        i = prob.oG + 4 * gi
        cov_g[l.name] = cov[i:i + 4, i:i + 4]
    cov_e = {}
    emap = {}
    for ei, ed in enumerate(prob.E):
        i = prob.oE + 2 * ei
        cov_e[ed.side] = cov[i:i + 2, i:i + 2]
        emap[ed.side] = e[ei]
    est = SM.Estimate(prob.L, prob.all_edges, g={l.name: g[gi] for gi, l in enumerate(prob.L)},
                      e={x.side: emap.get(x.side, np.zeros(2)) for x in prob.all_edges},
                      cov_g=cov_g, cov_e={x.side: cov_e.get(x.side, np.diag([1.0, 1.0])) for x in prob.all_edges},
                      cam_sys_cm=o["cam_sys_cm"] if cam_sys_cm is None else cam_sys_cm)
    est.extra = dict(edge_k_cm=float(k), edge_k_sd_cm=float(math.sqrt(cov[prob.oK, prob.oK])),
                     station_poses=X.tolist())
    robots_out = {}
    for ri, rn in enumerate(prob.robots):
        i = prob.oR + 7 * ri
        sd = np.sqrt(np.clip(np.diag(cov)[i:i + 7], 0, None))
        robots_out[rn] = dict(zone_offset_deg=[round(float(v), 3) for v in r[ri, :5]],
                              range_bias_cm=round(float(r[ri, 5]), 3), range_scale=round(1 + float(r[ri, 6]), 5),
                              bias_sigma_cm=round(float(sd[5]), 3), yaw_sigma_deg=round(float(np.mean(sd[:5])), 3),
                              zone_sd_deg=[round(float(v), 3) for v in sd[:5]], scale_sd=round(float(sd[6]), 5),
                              readings=int(np.sum(prob.sr[prob.obs["s"]] == ri)) if prob.obs is not None else 0)
    # what the readings say about their own noise (for the fusion's calibration file)
    noise = dict(a=o["tof_noise_cm"], b=o["tof_noise_rel"], width=round(prob.width(p), 3))
    n_tof = len(prob.obs["v"]) if prob.obs is not None else 0
    return Result(prob, p, cov, chi2, dof, n_tof, est, robots_out, noise)


def residual_table(res):
    """Per landmark: number of readings and their rms residual (cm) after the solve."""
    prob = res.prob
    if prob.obs is None or not len(prob.obs["v"]):
        return {}
    r = prob.obs["v"] - prob.predict_tof(res.p)
    out = {}
    for gi, l in enumerate(prob.L):
        m = prob.fg[prob.obs["f"]] == gi
        if m.any():
            out[l.name] = dict(n=int(m.sum()), rms_cm=round(float(np.sqrt(np.mean(r[m] ** 2))), 3),
                               mean_cm=round(float(np.mean(r[m])), 3))
    return out


def heading_guesses(sts):
    """Kept for callers: the start headings are set by observe.stations (heading_guess)."""
    for s in sts:
        if not hasattr(s, "heading_guess"):
            s.heading_guess = s.cam_th if s.cam_th is not None else 0.0


def build(landmarks, edges, runs, sightings, robots, opts=None, robot_priors=None, tape=None, min_cam=5):
    """Stations and links from the logs, then the Problem."""
    sts = OB.stations(runs, sightings, min_cam=min_cam)
    heading_guesses(sts)
    links = OB.odometry_links(sts)
    return Problem(landmarks, edges, sts, links, robots, opts, robot_priors, tape)
