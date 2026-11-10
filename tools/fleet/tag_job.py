#!/usr/bin/env python3
"""Mission Control's AprilTag screens (conference-table build, 2026-11-02): the server side of "Cameras & table" and
"Production", and the sync card. Used by mission_control.py when the site's tracking is "tags".

  room        the cameras' pictures: the Pi's recorder (robots), or a simulated room (two tag cameras over the site's
              table, the robots standing at their homes: tools/sim/tag_cam.py through fleet_sim.py) for practice
  check-out   camera_checkout.py on both cameras
  clicks      corners (1-4) and stickers (their numbers) clicked in each camera's picture; typed measurements.
              (2026-11-03) With reference tags on the table the corners come from the tags; corner clicking is a
              switch per site (the site's corner_clicks, changeable on the page; kept, never removed)
  refs        the reference tags (ref_tags.py) read in the pictures and fitted with everything else; their tiles
              become keep-out areas in the workspace map
  measure     tag_setup.py: the robots' tags read, site_fit.full, camera models saved to <conf_dir>/tagcal/, and the
              workspace map remade: the measured table, the production points as spots, the homes
  refit       site_fit.refit with the last run's tag reads (saved beside the run; "Use it" copies it in)
  production  production.program from the orders; planned like any program (missions.build, the 8 cm net)
  sync        the session id, the Pi's clock status, Start / End marks, the pose timeline, the comparison
"""
VERSION = "tag-job-2026-11-10e13"   # e13: corner tags pre-fill the corners, confirmed by the person (corner_confirm; MIRLT_CORNER_CONFIRM=auto: as before); Measure works with no roof tag when the corners are known. No version string before
import copy
import json
import os
import shutil
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
for p in (HERE, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "live"),
          os.path.join(TOOLS, "experiment")):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np    # noqa: E402

REAL_STREAMS = {"overhead": "127.0.0.1:8000", "side": "127.0.0.1:8001"}
SIM_PORTS = {"overhead": 8785, "side": 8786}       # the practice room's cameras (a run's own simulator uses
                                                   # 8765/8766, the corner setup's dry runs 8775/8776)


def tags_of(site):
    t = dict((site or {}).get("tags") or {})
    t.setdefault("family", "tag25h9")
    t.setdefault("tag_cm", 8.0)
    t.setdefault("roof_cm", 9.5)
    if t.get("height_cm") is None:
        t["height_cm"] = 6.0
        t["height_placeholder"] = True
    return t


class TagJob:
    def __init__(self, session, site):
        self.s = session
        self.site = site
        self.tags = tags_of(site)
        self.cal_dir = os.path.join(site["conf_dir"], "tagcal")
        self.state_path = os.path.join(session.dir, "tag_setup.json")
        self.d = {"clicks": {"overhead": {"corners": {}, "stickers": {}}, "side": {"corners": {}, "stickers": {}}},
                  "typed": {"table": dict(site.get("table") or {}), "stickers": {}, "distances": []},
                  "result": None, "checkout": None, "pictures": {}, "session_id": None, "refit": None,
                  "corner_clicks": None}
        try:
            with open(self.state_path) as f:
                self.d.update(json.load(f))
        except (OSError, ValueError):
            pass
        self.d.setdefault("typed", {}).setdefault("along", {})
        rt = site.get("ref_tags") or {}
        self.ref_cfg = rt if (rt and rt.get("use", True)) else None
        self.pics = {}            # camera -> (t, grey)
        self.room = None          # the practice simulator (subprocess)
        self.room_cfg = None
        self.busy = None
        self.lock = threading.Lock()

    def corner_clicks_on(self):
        """Corner clicking: the page's choice when made, else the site's corner_clicks (default on)."""
        v = self.d.get("corner_clicks")
        return bool(self.site.get("corner_clicks", True)) if v is None else bool(v)

    def set_corner_clicks(self, on):
        self.d["corner_clicks"] = bool(on)
        self.save()
        return {"corner_clicks": self.corner_clicks_on()}

    def save(self):
        with open(self.state_path, "w") as f:
            json.dump({k: v for k, v in self.d.items() if k != "pictures"}, f, indent=1, default=str)

    # -- the cameras ------------------------------------------------------------------------------------------
    def streams(self, conn):
        if conn == "robots":
            return dict(REAL_STREAMS)
        self.start_room()
        return {c: "127.0.0.1:%d" % p for c, p in SIM_PORTS.items()}

    def start_room(self):
        """The practice room: the simulator's two tag cameras over this site's table, robots at their homes."""
        if self.room is not None and self.room.poll() is None:
            return
        import tag_cam as TCAM
        import production as PR
        tb = self.d["typed"].get("table") or {}
        L, W = float(tb.get("length_cm") or 396.24), float(tb.get("width_cm") or 167.64)
        names = [r["name"] for r in self.site["robots"]]
        cfg = TCAM.default_config(table=(L, W), size=(self.site["capture"]["width"], self.site["capture"]["height"]),
                                  tags=self.tags, refs=self.ref_cfg or False)
        for k, c in enumerate(cfg["cams"]):
            c["port"] = list(SIM_PORTS.values())[k]
        d = os.path.join(self.s.dir, "practice-room")
        os.makedirs(d, exist_ok=True)
        cp = os.path.join(d, "tag_cams.json")
        with open(cp, "w") as f:
            json.dump(cfg, f, indent=1)
        import sticker_points as SP
        if SP.typed_on(self.site):
            m = PR.site_production(self.site, names, table=(L, W))[0]     # the site's homes (P11 / P22)
        else:
            m, _ = PR.site_map((L, W), names)
        cfg["room_robots"] = {r["name"]: r["start"] for r in m["robots"]}
        with open(cp, "w") as f:
            json.dump(cfg, f, indent=1)
        log = open(os.path.join(d, "room.txt"), "w")
        self.room = subprocess.Popen([sys.executable, os.path.join(TOOLS, "sim", "tag_cam.py"), "--config", cp,
                                      "--out", os.path.join(d, "sim")],
                                     cwd=REPO, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.room_cfg = cfg
        self.room_dir = d
        time.sleep(2.5)

    def stop_room(self):
        if self.room is not None and self.room.poll() is None:
            self.room.terminate()
            try:
                self.room.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self.room.kill()
        self.room = None

    def take_pictures(self, conn):
        import tag_setup as TSU
        pics = TSU.grab_pictures(self.streams(conn), 1.5, max_n=20)
        self.pics = {c: p[-1] for c, p in pics.items() if p}
        self.pics_all = pics
        self.d["pictures"] = {c: {"size": [int(g.shape[1]), int(g.shape[0])], "t": t} for c, (t, g) in self.pics.items()}
        if not self.pics:
            raise ValueError("no pictures from the cameras (%s)" % ("is the recorder running? ./run-fleet-pi.sh rec"
                                                                     if conn == "robots" else "the practice room"))
        return self.d["pictures"]

    def picture_jpeg(self, cam, marks=True):
        """The newest picture with what is clicked (corners red, stickers blue) and the tags read (green)."""
        import cv2
        if cam not in self.pics:
            return None
        t, g = self.pics[cam]
        img = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        if marks:
            import tagcore as TC
            for d in TC.TagDetector(self.tags["family"], ids=list(int(i) for i in self.tags["ids"].values())).detect_full(g):
                cv2.polylines(img, [np.round(d.corners).astype(np.int32)], True, (60, 160, 18), 2)
                cv2.putText(img, "tag %d" % d.id, tuple(int(v) for v in d.corners[0] + [0, -6]), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (60, 160, 18), 2)
            if self.ref_cfg:
                # the reference tags (orange): which corner or side each is, found by themselves (2026-11-03)
                import ref_tags as RT
                lay = RT.layout(self.ref_cfg)
                for d in RT.detector(self.ref_cfg).detect_full(g):
                    it = lay.get(str(d.id), {})
                    lab = ("corner %d" % it["corner"]) if it.get("kind") == "corner" else "side"
                    cv2.polylines(img, [np.round(d.corners).astype(np.int32)], True, (0, 140, 255), 3)
                    c = d.center
                    cv2.putText(img, "%d %s" % (d.id, lab), (int(c[0]) - 40, int(c[1]) + 5), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 110, 230), 2)
            cl = self.d["clicks"].get(cam) or {}
            for k, (u, v) in (cl.get("corners") or {}).items():
                cv2.circle(img, (int(u), int(v)), 9, (24, 35, 180), 2)
                cv2.putText(img, str(k), (int(u) + 10, int(v) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (24, 35, 180), 2)
            for k, (u, v) in (cl.get("stickers") or {}).items():
                cv2.circle(img, (int(u), int(v)), 8, (204, 82, 41), 2)
                cv2.putText(img, str(k), (int(u) + 9, int(v) + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (204, 82, 41), 2)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return buf.tobytes() if ok else None

    # -- clicks and typed -------------------------------------------------------------------------------------
    def set_clicks(self, d):
        cam = d["cam"]
        cl = self.d["clicks"].setdefault(cam, {"corners": {}, "stickers": {}})
        kind = d.get("kind", "corner")
        key = str(d.get("label", "")).strip().upper()
        if not key:
            raise ValueError("give the corner's number (1-4) or the sticker's number")
        if kind == "corner" and key not in ("1", "2", "3", "4"):
            raise ValueError("a corner is 1, 2, 3 or 4 (1 = 0,0; 1 -> 2 along x; 1 -> 4 along y, counter-clockwise "
                             "seen from above)")
        bucket = cl["corners" if kind == "corner" else "stickers"]
        if d.get("remove"):
            bucket.pop(key, None)
        else:
            bucket[key] = [float(d["u"]), float(d["v"])]
        self.save()
        return self.d["clicks"]

    def set_typed(self, d):
        t = self.d["typed"]
        if "table" in d:
            tb = d["table"] or {}
            t["table"] = {k: float(v) for k, v in tb.items() if k in ("length_cm", "width_cm") and v not in ("", None)}
        if "stickers" in d:
            t["stickers"] = {str(k).upper(): [float(a), float(b)] for k, (a, b) in (d["stickers"] or {}).items()}
        if "distances" in d:
            t["distances"] = [[str(a).upper(), str(b).upper(), float(c)] for a, b, c in (d["distances"] or [])]
        if "along" in d:
            # a side tag's place along its edge, taped (a check, 2026-11-03): {id: cm from the edge's first corner}
            t["along"] = {str(k): float(v) for k, v in (d["along"] or {}).items() if v not in ("", None)}
        self.save()
        return t

    # -- check-out --------------------------------------------------------------------------------------------
    def checkout(self, conn):
        import camera_checkout as CO
        import e02_camera as CAM
        models, area = {}, None
        if os.path.exists(os.path.join(self.cal_dir, "site_fit.json")) and conn == "robots":
            for c in ("overhead", "side"):
                p = os.path.join(self.cal_dir, c + ".json")
                if os.path.exists(p):
                    models[c] = CAM.load_camera(p)
            with open(os.path.join(self.cal_dir, "site_fit.json")) as f:
                L, W = json.load(f)["size_cm"]
            area = (0, 0, L, W)
        elif conn != "robots" and self.d.get("result"):
            area = (0, 0) + tuple(self.d["result"]["size_cm"])
            for c in ("overhead", "side"):
                p = os.path.join(self.sim_cal_dir(), c + ".json")
                if os.path.exists(p):
                    models[c] = CAM.load_camera(p)
        res = CO.run(self.streams(conn), tags=self.tags, models=models or None, area=area,
                     want_size=[self.site["capture"]["width"], self.site["capture"]["height"]],
                     labels={k: v.get("label", k) for k, v in self.site["cameras"].items()},
                     clicks={c: (v.get("corners") or {}) for c, v in self.d["clicks"].items()})
        res["words"] = CO.words(res)
        self.d["checkout"] = res
        self.save()
        return res

    def sim_cal_dir(self):
        return os.path.join(self.s.dir, "practice-room", "tagcal")

    # -- measure ----------------------------------------------------------------------------------------------
    # -- (e13) corner tags pre-fill the corner clicks; the person confirms them ----------------------------------
    def corner_confirm_mode(self):
        """"confirm" (e13, Nick 10 Nov: the corner tags PRE-FILL the corners and the person confirms them on the picture)
        or "auto" (before e13: the corner tags used with no confirm). MIRLT_CORNER_CONFIRM, else the site's
        corner_confirm, else confirm. The camera self-check's refit at a run's start passes confirm=False (nobody is
        at the page then; it is the old automatic way, as before)."""
        return os.environ.get("MIRLT_CORNER_CONFIRM") or str(self.site.get("corner_confirm") or "confirm")

    def corners_from_refs(self, refs):
        """{camera: {"1": [u, v], ...}}: the corner tiles' middles (ref_tags CORNER_IDS: ids 20-23 = corners 1-4)."""
        import numpy as np
        cids = {str(v): k for k, v in ((self.ref_cfg or {}).get("corners") or {}).items()}
        out = {}
        for c, got in (refs or {}).items():
            for i, cs in got.items():
                if str(i) in cids:
                    out.setdefault(c, {})[str(cids[str(i)])] = [round(float(v), 1) for v in np.mean(np.asarray(cs), 0)]
        return out

    def confirm_corners(self, proposal):
        """The person confirmed the corners found (the proposal as shown): kept; the next Measure uses them."""
        self.d["corners_ok"] = {"corners": proposal, "t": time.time()}
        self.save()
        return {"confirmed": True}

    def _confirmed(self, proposal, tol_px=6.0):
        ok = (self.d.get("corners_ok") or {}).get("corners") or {}
        if not ok or set(ok) != set(proposal):
            return False
        for c, pts in proposal.items():
            for k, uv in pts.items():
                q = (ok.get(c) or {}).get(k)
                if q is None or abs(q[0] - uv[0]) > tol_px or abs(q[1] - uv[1]) > tol_px:
                    return False
        return True

    def measure(self, conn, confirm=None):
        import tag_setup as TSU
        import production as PR
        streams = self.streams(conn)
        pics = TSU.grab_pictures(streams, TSU.PICTURE_S)
        reads, ms = TSU.read_tags(pics, self.tags)
        refs = None
        if self.ref_cfg:
            import ref_tags as RT
            refs = RT.read_refs(pics, self.ref_cfg)
        clicked = any(((self.d["clicks"].get(c) or {}).get("corners")) for c in self.d["clicks"])
        if not reads and not (refs and any(refs.values())) and not clicked:
            # (e13) no roof tags is not a refusal when the corners are known (reference tags or clicks)
            raise ValueError("no robot's tag and no reference tag was read, and no corner is clicked: click the "
                             "table's corners in the pictures (or put the reference tiles out), then Measure again")
        if confirm is None:
            confirm = self.corner_confirm_mode() == "confirm"
        prop = self.corners_from_refs(refs) if (refs and any(refs.values())) else {}
        if confirm and prop and not self.corner_clicks_on() and not self._confirmed(prop):
            self.d["corners_proposed"] = {"corners": prop, "t": time.time(), "conn": conn}
            self.save()
            return {"confirm_corners": prop, "words": [
                "The corner tags were found: check that each numbered dot sits on the table's corner in the picture, "
                "then press 'These are the corners' (or click them yourself)."]}
        sizes = {c: [int(p[-1][1].shape[1]), int(p[-1][1].shape[0])] for c, p in pics.items() if p}
        if self.corner_clicks_on():
            corners = {c: dict((self.d["clicks"].get(c) or {}).get("corners") or {}) for c in sizes}
        else:
            corners = {c: {} for c in sizes}       # switched off: the reference tags give the corners
            if not (refs and any(refs.values())):
                raise ValueError("no reference tag was read and corner clicking is off: put the reference tiles on the "
                                 "table's corners and sides (ids 20-25), or switch corner clicking on and click the "
                                 "corners")
        st = {}
        for c in sizes:
            for k, uv in ((self.d["clicks"].get(c) or {}).get("stickers") or {}).items():
                st.setdefault(k, {"cams": {}})["cams"][c] = uv
        for k, xy in (self.d["typed"].get("stickers") or {}).items():
            if k in st:
                st[k]["xy_cm"] = xy
        tb = self.d["typed"].get("table") or {}
        obs = TSU.make_obs(sizes, corners, st, self.tags, TSU.groups_from_reads(reads, still=True),
                           table=tb if tb.get("length_cm") else None, distances=self.d["typed"].get("distances"),
                           refs=refs, ref_cfg=self.ref_cfg, typed_along=self.d["typed"].get("along"))
        out_dir = self.cal_dir if conn == "robots" else self.sim_cal_dir()
        if conn == "robots" and os.path.isdir(out_dir):
            bk = out_dir + "-before-" + time.strftime("%Y%m%d-%H%M%S")
            shutil.copytree(out_dir, bk)
        res = TSU.fit_and_save(obs, out_dir)
        words = TSU.result_words(res)
        if self.tags.get("height_placeholder"):
            words.insert(0, "The roof's height is NOT measured yet: 6.0 cm used. Measure from the table to the tag's "
                            "face and put it in the site profile (tags.height_cm), then Measure again.")
        L, W = res["size_cm"]
        names = [r["name"] for r in self.site["robots"]]
        kos = None
        if res.get("ref_tags"):
            import ref_tags as RT
            kos = RT.keepouts(res["ref_tags"]["poses"], self.ref_cfg)
        import sticker_points as SP
        if SP.typed_on(self.site):
            # typed points (Jay's sheet, 2026-11-03): the fit fixes the frame and the table's size; the typed spots are
            # the production points; a sticker the cameras put elsewhere (clicked) is flagged, never moved
            m, moved, prog, orders = PR.site_production(self.site, names, table=(L, W), keepouts=kos,
                                                        seen=res["stickers"])
            self.s.program = prog
            self.s.last_production = {"orders": orders, "dwell_s": prog["production"]["dwell_s"], "home": True}
            words.append("Production points: the %d typed stickers (%s); their spots are not clicked" % (
                len(m.get("spots") or {}), os.path.basename(SP.points_file(self.site))))
        else:
            m, moved = PR.site_map((L, W), names, points=res["stickers"], keepouts=kos)
        words += moved
        self.s.map = m
        self.s.save_map()
        self.d["result"] = {"size_cm": res["size_cm"], "words": words, "stickers": (m.get("spots") if SP.typed_on(self.site) else res["stickers"]),
                            "rms_px": res["rms_px"], "checks": res.get("checks"), "tag_scale": res.get("tag_scale"),
                            "ref_tags": res.get("ref_tags"), "height_check": res.get("height_check"),
                            "roof_tags": res.get("roof_tags"),      # (2026-11-03) each robot's roof: height, tilt
                            "corner_clicks": self.corner_clicks_on(),
                            "t": time.time(), "conn": conn, "saved_to": out_dir, "reads": len(reads)}
        if conn != "robots" and self.room_cfg:
            truth = {}
            from cameras import CameraModel
            for c in ("overhead", "side"):
                p = os.path.join(self.room_dir, "sim", "true_%s.json" % c)
                if os.path.exists(p):
                    truth[c] = CameraModel.load(p)
            import site_fit as SF
            self.d["result"]["sim_truth"] = SF.compare(res, truth, {k: tuple(v) for k, v in self.room_cfg["stickers"].items()},
                                                       H=float(self.tags["height_cm"]), table=(L, W))
        self.save()
        return self.d["result"]

    def _refit_inputs(self, run_folder):
        ob = os.path.join(self.cal_dir if not run_folder or "-sim" not in run_folder else self.sim_cal_dir(), "obs.json")
        to = os.path.join(run_folder, "bridge", "tag_obs.csv")
        for cand in (os.path.join(run_folder, "tagcal", "obs.json"), ob):
            if os.path.exists(cand):
                ob = cand
                break
        return ob, to

    def refit_live(self, run_folder):
        """The last run's refit: none / running (seconds so far) / done / failed (site_fit.refit_status)."""
        import site_fit as SF
        if not run_folder:
            return {"state": "none"}
        return SF.refit_status(os.path.join(run_folder, "tagcal_refit"))

    def refit(self, run_folder, use=False, wait=False):
        """The Refit button (2026-11-03): the run's own refit is REUSED when it is done (the run starts it in the
        background); while it runs, says so; when there is none (auto refit off, or it failed), starts one in the
        background and returns at once. wait=True: the old way (computed here, the request waits for it)."""
        import site_fit as SF
        ob, to = self._refit_inputs(run_folder)
        out = os.path.join(run_folder, "tagcal_refit")
        st = SF.refit_status(out) if run_folder else {"state": "none"}
        if st["state"] == "running":
            return {"running": True, "folder": out, "words": [
                "The refit from this run is still running in the background (%.0f s so far; a few minutes on the Pi). "
                "This card fills in by itself when it is done; then Use the refit." % st.get("seconds", 0)]}
        if st["state"] != "done" or wait:
            if not (os.path.exists(ob) and os.path.exists(to)):
                raise ValueError("no setup or no tag reads to refit with (%s, %s)" % (ob, to))
            if not wait:
                SF.start_refit_bg(ob, to, out)
                return {"running": True, "folder": out, "words": [
                    "Refit started in the background (a few minutes on the Pi%s). This card fills in by itself when it "
                    "is done." % ("; the run's own refit had failed: %s" % st.get("why") if st["state"] == "failed"
                                  else "")]}
            res = SF.refit(ob, to, out)
            st = {"state": "done", "size_cm": res["size_cm"], "words": res["words"], "rms_px": res["rms_px"]}
        self.d["refit"] = {"folder": out, "size_cm": st.get("size_cm"), "words": st.get("words") or [],
                           "rms_px": st.get("rms_px"), "reused": not wait}
        if use:
            bk = self.cal_dir + "-before-" + time.strftime("%Y%m%d-%H%M%S")
            if os.path.isdir(self.cal_dir):
                shutil.copytree(self.cal_dir, bk)
            os.makedirs(self.cal_dir, exist_ok=True)
            for f in os.listdir(out):
                if f in ("status.json", "refit.txt") or f.endswith(".tmp"):
                    continue          # the background refit's own notes stay with the run
                shutil.copy(os.path.join(out, f), os.path.join(self.cal_dir, f))
            self.d["refit"]["used"] = True
        self.save()
        return self.d["refit"]

    # -- production -------------------------------------------------------------------------------------------
    def experiment_preview(self, d):
        """The Experiment picker (2026-11-03, Jay-Sun's training runs): the chosen experiment's orders on the map in use,
        whether the halves are swapped (auto: the site's run counter), crossings of the middle, pinned speeds."""
        import experiments as EX
        import sticker_points as SP
        exp = str(d.get("experiment") or "custom")
        mode = str(d.get("swap_halves") or self.site.get("experiments", {}).get("swap_halves") or "auto")
        sw, words = EX.swap_for(self.site, mode) if exp not in ("custom", "shakedown") else (
            False, "halves as usual" + (" (shakedown)" if exp == "shakedown" else " (your own orders)"))
        out = {"experiment": exp, "swap_mode": mode, "swapped": sw, "swap_words": words,
               "choices": [{"id": k, "name": v["name"]} for k, v in EX.KINDS.items()] +
               [{"id": "custom", "name": "Custom: the orders typed below"}],
               "wiggle": EX.wiggle_on(self.site), "speed": EX.conf(self.site)["speed"],
               "runs_so_far": EX.counter_read(self.site)["runs"]}
        if exp == "custom" or not SP.typed_on(self.site):
            return out
        spots = self.s.map.get("spots") or {}
        _p, zones, _w = SP.points_cm(self.site)
        homes = {r["name"]: r["start"] for r in self.s.map.get("robots", [])}
        orders, meta = EX.experiment_orders(self.site, spots, zones, homes, exp, swap=sw)
        meta["swap_mode"], meta["swap_words"] = mode, words
        out.update(orders=orders, meta=meta, crossings=meta["crossings"])
        return out

    def production_program(self, d):
        import production as PR
        exp = str(d.get("experiment") or "custom")
        meta = None
        if exp != "custom":
            # an experiment picked (2026-11-03): its orders replace the typed lists (the custom orders stay behind it)
            pv = self.experiment_preview(d)
            if pv.get("orders"):
                d = dict(d, orders=pv["orders"])
                meta = pv["meta"]
        orders = {n: [str(x).strip().upper() for x in v if str(x).strip()] for n, v in (d.get("orders") or {}).items()}
        orders = {n: v for n, v in orders.items() if v or d.get("home", True)}
        bad = PR.check_orders(orders, self.s.map.get("spots") or {})
        if bad:
            raise ValueError("; ".join(bad))
        # homes at points (2026-11-03): a robot's home point first in its list is stood at, not driven to
        prog = PR.program(orders, dwell_s=float(d.get("dwell_s", PR.DWELL_S)), home=bool(d.get("home", True)),
                          at_home=PR.site_home_points(self.site))
        if d.get("sync_spin"):
            prog = add_sync_spin(prog)
        if meta is not None:
            prog["production"]["experiment"] = meta
            prog["name"] = "production %s%s" % (exp, " (halves swapped)" if meta.get("swap_halves") else "")
        import experiments as EX
        if EX.wiggle_on(self.site, {True: "on", False: "off"}.get(d.get("sync_wiggle"))):
            prog = EX.add_wiggle(prog, self.site)        # the sync wiggle (Nick, 3 Nov): the site's switch, or the box
        return prog

    # -- sync -------------------------------------------------------------------------------------------------
    def sync_state(self):
        import sync_tools as ST
        if not self.d.get("session_id"):
            self.d["session_id"] = ST.session_id(self.site["name"])
            self.save()
        c = getattr(self, "_clock", None)
        if c is None or time.time() - c[0] > 30:
            self._clock = (time.time(), ST.clock_status())
        return {"session_id": self.d["session_id"], "clock": self._clock[1],
                "marks": ST.read_marks(self.mark_folder())[-10:]}

    def new_session_id(self):
        import sync_tools as ST
        self.d["session_id"] = ST.session_id(self.site["name"])
        self.save()
        return self.sync_state()

    def mark_folder(self):
        run = self.s.run
        if run is not None and run.alive():
            return run.folder
        return os.path.join(self.s.dir, "sync")

    def mark(self, what):
        import sync_tools as ST
        m = ST.mark(self.mark_folder(), what, session=self.d.get("session_id"))
        return m


def add_sync_spin(prog):
    """Each robot spins once on the spot at the start (two half turns at its home): a moment both recordings see."""
    out = copy.deepcopy(prog)
    pre = []
    for n in out["robots"]:
        pre.append({"block": "sync-spin", "robot": n})
    out["blocks"] = pre + out["blocks"]
    out["sync_spin"] = True
    return out
