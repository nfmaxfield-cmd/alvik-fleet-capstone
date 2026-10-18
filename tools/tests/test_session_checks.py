"""The automatic per-session checks (tools/session_checks.py, run by
tools/check_session.py) on small synthetic data: each check must pass clean
data and catch the fault it is there for (a camera that moved, light that
changed, a drifting video clock, dropped frames, WiFi bursts, rest noise,
gyro warm-up, a lifted robot, a slow distance sensor). A few seconds."""
import csv
import json
import math
import os
import random

import numpy as np
import pytest

import check_session as cs
import session_checks as sc

cv2 = pytest.importorskip("cv2")

W, H, FPS = 320, 240, 25.0


# ------------------------------------------------------------------ synthetic recordings
SS = 4      # the floor is drawn 4x finer, moved, then shrunk: a true sub-pixel camera move (no interpolation blur)


def floor_view(seed=0, shift=(0.0, 0.0), rot_deg=0.0):
    """Plywood-like texture (blotches and finer grain) with two tape lines, as
    the camera sees it after moving by `shift` px and turning rot_deg about
    the image centre."""
    rng = np.random.default_rng(seed)
    pad = 16 * SS
    h, w = H * SS + 2 * pad, W * SS + 2 * pad

    def comp(sig, sd):
        c = cv2.GaussianBlur(rng.normal(0, 1, (h, w)).astype(np.float32), (0, 0), sig * SS)
        return c / c.std() * sd
    big = comp(6, 25) + comp(1.5, 8) + 130
    cv2.line(big, (pad + 20 * SS, pad + 120 * SS), (pad + 300 * SS, pad + 125 * SS), 40, 4 * SS)
    cv2.line(big, (pad + 60 * SS, pad + 10 * SS), (pad + 62 * SS, pad + 230 * SS), 40, 4 * SS)
    M = cv2.getRotationMatrix2D((w / 2, h / 2), rot_deg, 1.0)
    M[0, 2] += shift[0] * SS
    M[1, 2] += shift[1] * SS
    big = cv2.warpAffine(big, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
    return cv2.resize(big[pad:pad + H * SS, pad:pad + W * SS], (W, H), interpolation=cv2.INTER_AREA)


def write_recording(folder, seconds=8.0, shift_after_s=None, shift=(0.0, 0.0), rot_deg=0.0, light_after_s=None,
                    light=1.0, seed=0, meta_extra=None):
    """A camrec-style folder: video.avi (MJPG), frames.csv, meta.json. A bright
    square (a robot) drives across; after shift_after_s the camera has moved
    by `shift` px and turned by rot_deg; after light_after_s the brightness is
    multiplied by `light`."""
    os.makedirs(folder, exist_ok=True)
    base = floor_view(seed)
    moved = floor_view(seed, shift, rot_deg) if shift_after_s is not None else base
    rng = np.random.default_rng(seed + 1)
    n = int(seconds * FPS)
    vw = cv2.VideoWriter(os.path.join(folder, "video.avi"), cv2.VideoWriter_fourcc(*"MJPG"), FPS, (W, H))
    t0 = 1792332900.0
    with open(os.path.join(folder, "frames.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "t_wall", "t_mono", "cam_ms"])
        for k in range(n):
            t = k / FPS
            img = (moved if shift_after_s is not None and t >= shift_after_s else base).copy()
            x = int(30 + 250 * (t / seconds))
            cv2.rectangle(img, (x, 150), (x + 30, 180), 230, -1)       # the "robot"
            if light_after_s is not None and t >= light_after_s:
                img = img * light
            img = np.clip(img + rng.normal(0, 2, img.shape), 0, 255).astype(np.uint8)
            vw.write(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
            w.writerow([k, f"{t0 + t + 0.04:.4f}", f"{t:.4f}", f"{1000 * t + 5000:.1f}"])
    vw.release()
    meta = dict(device="synthetic", width=W, height=H, fps_requested=25, fps_reported=25.0, exposure_ms=12.0)
    meta.update(meta_extra or {})
    with open(os.path.join(folder, "meta.json"), "w") as f:
        json.dump(meta, f)
    return folder


@pytest.fixture(scope="module")
def recs(tmp_path_factory):
    root = tmp_path_factory.mktemp("recs")
    return dict(
        clean=write_recording(str(root / "clean")),
        shifted=write_recording(str(root / "shifted"), shift_after_s=4.0, shift=(0.8, -0.5)),
        turned=write_recording(str(root / "turned"), shift_after_s=4.0, rot_deg=0.25),
        tiny=write_recording(str(root / "tiny"), shift_after_s=4.0, shift=(0.15, 0.1)),
        darker=write_recording(str(root / "darker"), light_after_s=5.0, light=0.8),
    )


def by_check(results, check):
    out = [r for r in results if r.check == check]
    assert out, f"no {check} result"
    return out[0]


# ------------------------------------------------------------------ camera moved, lighting
def test_still_camera_passes(recs):
    res = sc.camera_still_and_light_checks(recs["clean"], px_per_cm=3.4)
    cam = by_check(res, "camera_moved")
    assert cam.status == "PASS" and cam.value < 0.1, cam.line()
    assert by_check(res, "lighting").status == "PASS"


def test_camera_shift_is_caught(recs):
    cam = by_check(sc.camera_still_and_light_checks(recs["shifted"]), "camera_moved")
    assert cam.status in ("CHECK", "FAIL"), cam.line()
    assert abs(cam.value - math.hypot(0.8, 0.5)) < 0.1, cam.line()      # measured shift ~ the true 0.94 px


def test_camera_turn_is_caught(recs):
    cam = by_check(sc.camera_still_and_light_checks(recs["turned"]), "camera_moved")
    assert cam.status in ("CHECK", "FAIL"), cam.line()
    assert "0.25" in cam.detail or "0.24" in cam.detail or "0.26" in cam.detail, cam.line()


def test_small_shift_measured_but_passes(recs):
    cam = by_check(sc.camera_still_and_light_checks(recs["tiny"]), "camera_moved")
    assert cam.status == "PASS" and 0.1 < cam.value < 0.3, cam.line()


def test_lighting_change_is_caught(recs):
    lit = by_check(sc.camera_still_and_light_checks(recs["darker"]), "lighting")
    assert lit.status == "FAIL" and abs(lit.value - 20) < 3, lit.line()


@pytest.mark.parametrize("shift,rot", [((1.3, -0.7), 0.0), ((0.25, 0.0), 0.0), ((6.0, 4.0), 0.0), ((0.0, 0.0), 0.3)])
def test_register_recovers_known_move(shift, rot):
    a = floor_view(3)
    b = floor_view(3, shift, rot)
    warp, n = sc.register(a, b, np.ones_like(a, bool))
    sh, r, _ = sc.warp_motion(warp, W, H)
    assert abs(sh - math.hypot(*shift)) < 0.05 and abs(abs(r) - rot) < 0.01, (sh, r)


# ------------------------------------------------------------------ settings, exposure
def test_camera_settings_change_flagged(tmp_path):
    ctl = dict(auto_exposure=1.0, exposure=120.0, gain=0.0)
    end = dict(ctl, exposure=300.0)
    d = write_recording(str(tmp_path / "r"), seconds=1.0, meta_extra=dict(camera_controls=dict(after=ctl),
                                                                         camera_controls_end=end))
    r = by_check(sc.camera_settings_check(d), "camera_settings")
    assert r.status == "CHECK" and "exposure 120.0 -> 300.0" in r.detail
    with open(os.path.join(d, "meta.json")) as f:
        m = json.load(f)
    m["camera_controls_end"] = ctl
    with open(os.path.join(d, "meta.json"), "w") as f:
        json.dump(m, f)
    assert by_check(sc.camera_settings_check(d), "camera_settings").status == "PASS"


def test_blink_reads_exposure_from_meta(tmp_path):
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(sc.__file__), "blink"))
    import blink
    (tmp_path / "meta.json").write_text(json.dumps(dict(exposure_ms=8.5)))
    assert blink.exposure_from_meta(str(tmp_path)) == (0.0085, "meta.json")
    (tmp_path / "meta.json").write_text(json.dumps(dict(fps_reported=25)))
    assert blink.exposure_from_meta(str(tmp_path)) == (blink.EXPOSURE_FALLBACK_S, "assumed")


def test_fps_fallback_reads_meta(tmp_path):
    (tmp_path / "meta.json").write_text(json.dumps(dict(fps_requested=25, fps_reported=25.0)))
    assert sc.nominal_fps(str(tmp_path)) == 25.0
    (tmp_path / "meta.json").write_text("{}")
    assert sc.nominal_fps(str(tmp_path)) is None


# ------------------------------------------------------------------ frame timing
def write_frames(folder, n=2000, fps=25.0, drops=(), late=(), jitter_ms=1.0, cam=True, seed=0):
    rng = random.Random(seed)
    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, "frames.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "t_wall", "t_mono", "cam_ms"] if cam else ["frame", "t_wall", "t_mono"])
        k = 0
        for i in range(n):
            if i in drops:
                continue
            tc = 1000.0 + i / fps
            tw = tc + 0.03 + abs(rng.gauss(0, jitter_ms / 1000)) + (0.025 if i in late else 0.0)
            row = [k, f"{tw:.4f}", f"{tc - 1000:.4f}"] + ([f"{tc * 1000 - 990000:.1f}"] if cam else [])
            w.writerow(row)
            k += 1
    with open(os.path.join(folder, "meta.json"), "w") as f:
        json.dump(dict(fps_requested=25, fps_reported=fps), f)
    return folder


def test_frame_timing_counts_drops_not_late_frames(tmp_path):
    d = write_frames(str(tmp_path / "a"), drops={100, 500, 501, 900}, late={300, 1200, 1500})
    res = sc.frame_timing_checks(d)
    drop = by_check(res, "frame_drops")
    assert drop.detail.startswith("4 frame(s) missing"), drop.line()
    assert by_check(res, "frame_rate").status == "PASS"
    j = by_check(res, "frame_jitter")
    assert j.status == "PASS" and "cam_ms" in j.detail


def test_frame_jitter_flagged_without_camera_timestamps(tmp_path):
    d = write_frames(str(tmp_path / "b"), jitter_ms=9.0, cam=False)
    j = by_check(sc.frame_timing_checks(d), "frame_jitter")
    assert j.status in ("CHECK", "FAIL") and "proxy" in j.detail, j.line()


def test_many_drops_fail(tmp_path):
    d = write_frames(str(tmp_path / "c"), drops=set(range(10, 2000, 50)))
    assert by_check(sc.frame_timing_checks(d), "frame_drops").status == "FAIL"


# ------------------------------------------------------------------ video clock drift
def test_video_clock_drift(tmp_path):
    rng = random.Random(1)
    ts = [1000.0 + 60 * i for i in range(10)]
    flat = [(t, 0.07 + rng.gauss(0, 0.003)) for t in ts]
    r = sc.video_clock_check("v", flat, write_to=str(tmp_path))[0]
    assert r.status == "PASS", r.line()
    j = json.loads((tmp_path / "video_clock.json").read_text())
    assert abs(j["offset_s"] - 0.07) < 0.003 and not j["use_drift"]
    drifting = [(t, 0.07 + 60e-6 * (t - ts[0]) + rng.gauss(0, 0.002)) for t in ts]     # 60 ppm: 32 ms in 9 min
    r = sc.video_clock_check("v", drifting, write_to=str(tmp_path))[0]
    assert r.status == "FAIL" and abs(r.value - 32.4) < 5, r.line()
    j = json.loads((tmp_path / "video_clock.json").read_text())
    assert j["use_drift"] and abs(j["drift_ppm"] - 60) < 8
    t = ts[0] + 100
    assert abs(j["offset_s"] + j["drift_ppm"] * 1e-6 * (t - j["t_ref"]) - (0.07 + 60e-6 * 100)) < 0.004
    noisy = [(t, 0.07 + rng.choice((-0.015, 0.015))) for t in ts]
    assert sc.video_clock_check("v", noisy)[0].status in ("CHECK", "FAIL")
    assert sc.video_clock_check("v", flat[:1])[0].status == "SKIP"


# ------------------------------------------------------------------ robot logs
def raw_rows(robot="Left", run="AB12-1", secs=20.0, hz=50, move=((5.0, 10.0),), rpm_noise=0.05, gz_bias=0.02,
             gz_drift_per_s=0.0, tilt=(0.0, 0.0), lifted=(), lost=(), tof_every=3, t0_ms=60000, seed=0):
    """RAW rows: still except during `move` (start, end s), when both wheels turn
    at 30 rpm. lost: row indices dropped; lifted: (start, end) s."""
    rng = random.Random(seed)
    out, wl, tof = [], 0.0, None
    n = int(secs * hz)
    for i in range(n):
        t = i / hz
        moving = any(a <= t < b for a, b in move)
        rpm = 30.0 if moving else 0.0
        wl += rpm * 6 / hz
        if i % tof_every == 0 or tof is None:
            tof = [round(100 + rng.gauss(0, 0.5), 1) for _ in range(7)]
        if i in lost:
            continue
        ax, ay = math.sin(math.radians(tilt[0])), math.sin(math.radians(tilt[1]))
        r = dict(robot=robot, run=run, ms=str(t0_ms + int(1000 * t)), step="1" if moving else "0",
                 wl_deg=f"{wl // 0.36 * 0.36:.2f}", wr_deg=f"{wl // 0.36 * 0.36:.2f}",
                 wl_rpm=f"{rpm + rng.gauss(0, rpm_noise):.2f}", wr_rpm=f"{rpm + rng.gauss(0, rpm_noise):.2f}",
                 v_cms=f"{rpm * 0.178 + rng.gauss(0, 0.01):.2f}", w_dps=f"{rng.gauss(0, 0.05):.2f}",
                 ax=f"{ax + rng.gauss(0, 0.003):.4f}", ay=f"{ay + rng.gauss(0, 0.003):.4f}",
                 az=f"{math.sqrt(max(0, 1 - ax * ax - ay * ay)) + rng.gauss(0, 0.003):.4f}",
                 gx=f"{rng.gauss(0, 0.07):.3f}", gy=f"{rng.gauss(0, 0.07):.3f}",
                 gz=f"{gz_bias + gz_drift_per_s * (t0_ms / 1000 + t) + rng.gauss(0, 0.07):.3f}",
                 lifted="1" if any(a <= t < b for a, b in lifted) else "0")
        for c, v in zip(sc.TOF_ZONES, tof):
            r[c] = f"{v:.1f}"
        out.append(r)
    return out


def test_still_windows_found():
    rr = raw_rows(move=((5.0, 10.0),))
    w = sc.still_windows(rr)
    assert len(w) == 2
    spans = [(float(rr[a]["ms"]) - 60000, float(rr[b]["ms"]) - 60000) for a, b in w]
    assert spans[0][0] >= 150 and abs(spans[0][1] - 4800) < 60 and abs(spans[1][0] - 10200) < 60


def test_rest_noise_against_fusion_threshold():
    quiet = sc.rest_noise_checks({("Left", "a-1"): raw_rows(rpm_noise=0.05)})[0]
    assert quiet.status == "PASS", quiet.line()
    noisy = sc.rest_noise_checks({("Left", "a-1"): raw_rows(rpm_noise=0.4)})[0]    # what tools/sim assumes
    assert noisy.status == "FAIL" and noisy.value < 50, noisy.line()


def test_tilt_and_gravity():
    flat = sc.tilt_checks({("Left", "a-1"): raw_rows(tilt=(0.5, 0.3))})
    assert by_check(flat, "tilt").status == "PASS" and abs(by_check(flat, "tilt").value - 0.58) < 0.15
    assert by_check(flat, "gravity").status == "PASS"
    tilted = by_check(sc.tilt_checks({("Left", "a-1"): raw_rows(tilt=(2.5, 1.0))}), "tilt")
    assert tilted.status == "CHECK" and abs(tilted.value - 2.69) < 0.2


def test_gyro_bias_warmup():
    rows = {("Left", "AB12-1"): raw_rows(move=((5, 10), (20, 25)), secs=40, gz_drift_per_s=0.01, t0_ms=0)}
    r = sc.gyro_bias_checks(rows)[0]
    assert r.status in ("CHECK", "FAIL") and r.value > 0.25, r.line()      # ~0.3 deg/s change over ~30 s
    rows = {("Left", "AB12-1"): raw_rows(move=((5, 10), (20, 25)), secs=40, t0_ms=0)}
    assert sc.gyro_bias_checks(rows)[0].status == "PASS"


def test_lifted_flagged():
    r = sc.lifted_checks({("Left", "a-1"): raw_rows(lifted=((12.0, 13.0),))})[0]
    assert r.status == "CHECK" and r.value == 50
    assert sc.lifted_checks({("Left", "a-1"): raw_rows()})[0].status == "PASS"


def test_tof_frame_rate():
    r = sc.tof_checks({("Left", "a-1"): raw_rows(tof_every=3)})[0]
    assert r.status == "PASS" and abs(r.value - 50 / 3) < 1 and "over 3 RAW row(s)" in r.detail, r.line()
    slow = sc.tof_checks({("Left", "a-1"): raw_rows(tof_every=15)})[0]      # 3.3 frames/s, 300 ms old
    assert slow.status == "FAIL", slow.line()


def test_wifi_bursts_vs_library_pauses():
    lost = set(range(400, 408))                         # 8 rows lost while moving (t = 8 s)
    rr = raw_rows(lost=lost)
    r = sc.wifi_checks(("Left", "a-1"), rr, 1000)
    assert by_check(r, "wifi_bursts").value == 8 and "1 burst(s)" in by_check(r, "wifi_loss").detail
    pause = set(range(100, 108))                        # same hole while sitting still (t = 2 s): a library pause
    r = sc.wifi_checks(("Left", "a-1"), raw_rows(lost=pause), 1000)
    assert by_check(r, "wifi_bursts").value == 0 and "1 gap(s) look like" in by_check(r, "wifi_loss").detail
    big = set(range(400, 420))                          # 420 ms hole: over the 300 ms rule
    assert by_check(sc.wifi_checks(("Left", "a-1"), raw_rows(lost=big), 1000), "wifi_loss").status != "PASS"


def test_clock_check_flags_unmeasured_drift():
    fits = {("Left", "AB12"): (0.0, 1.0, dict(n=3, bursts=1, used=1, rtt_med=10.0, rtt_max=12.0, drift_ppm=0.0,
                                               resid_ms=0.0)),
            ("Center", "CD34"): (0.0, 1 / (1 + 20e-6), dict(n=30, bursts=10, used=10, rtt_med=10.0, rtt_max=30.0,
                                                            drift_ppm=20.0, resid_ms=2.0))}
    r = {x.scope: x for x in sc.clock_checks(fits)}
    assert r["Left boot AB12"].status == "CHECK" and "NOT measured" in r["Left boot AB12"].detail
    assert r["Center boot CD34"].status == "PASS"


# ------------------------------------------------------------------ end to end
def write_session(folder, rr):
    os.makedirs(folder, exist_ok=True)

    def put(name, cols, rows):
        with open(os.path.join(folder, name + ".csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
            w.writeheader()
            for r in rows:
                w.writerow(r)
    raw_cols = ("robot,t_recv,run,ms,step,wl_deg,wr_deg,wl_rpm,wr_rpm,v_cms,w_dps,ax,ay,az,gx,gy,gz,"
                + ",".join(sc.TOF_ZONES) + ",lifted").split(",")
    put("raw", raw_cols, [dict(r, t_recv="0") for r in rr])
    put("runinfo", "robot,t_recv,run,experiment,fw_version,motor_fw,ms_start,battery".split(","),
        [dict(robot="Left", t_recv=0, run="AB12-1", experiment="demo", fw_version="x", motor_fw="1", ms_start=60000,
              battery=90)])
    put("runend", "robot,t_recv,run,result,ms_end,duration_ms,battery,raw_rows_sent".split(","),
        [dict(robot="Left", t_recv=0, run="AB12-1", result="done", ms_end=80000, duration_ms=20000, battery=90,
              raw_rows_sent=1000)])
    put("clock_sync", "robot,boot,token,t_send,t_recv,robot_ms,rtt_ms,offset_s".split(","),
        [dict(robot="Left", boot="AB12", token=f"c{i}", t_send=1792332840 + 10 * i, t_recv=1792332840.01 + 10 * i,
              robot_ms=60000 + 10000 * i - 5, rtt_ms=10, offset_s=0) for i in range(6)])


def test_check_session_writes_every_check(tmp_path, recs, capsys):
    s = str(tmp_path / "session")
    write_session(s, raw_rows(run="AB12-1", lifted=((15.0, 15.5),)))
    assert cs.main([s, "--video", recs["clean"]]) == 0
    out = capsys.readouterr().out
    assert "CHECKS" in out and "PASS" in out
    with open(os.path.join(s, "session_checks.csv"), newline="") as f:
        got = list(csv.DictReader(f))
    assert list(got[0].keys()) == sc.CSV_COLS
    ids = {r["check"] for r in got}
    for need in ("wifi_loss", "wifi_bursts", "robot_clock", "tilt", "gravity", "rest_noise", "gyro_bias", "lifted",
                 "tof_rate", "video_clock_drift", "frame_drops", "frame_rate", "frame_jitter", "exposure",
                 "camera_settings", "camera_moved", "lighting", "sync_flash_length", "blink_flash_length"):
        assert need in ids, need
    st_ = {r["check"]: r["status"] for r in got}
    assert st_["lifted"] == "CHECK" and st_["camera_moved"] == "PASS" and st_["video_clock_drift"] == "SKIP"
    assert all(r["status"] in ("PASS", "CHECK", "FAIL", "SKIP", "INFO") for r in got)


def test_check_session_without_video_reports_skips(tmp_path, capsys):
    s = str(tmp_path / "session")
    write_session(s, raw_rows(run="AB12-1"))
    cs.main([s])
    with open(os.path.join(s, "session_checks.csv"), newline="") as f:
        got = {r["check"]: r for r in csv.DictReader(f)}
    assert got["camera_moved"]["status"] == "SKIP" and got["video_clock_drift"]["status"] == "SKIP"
