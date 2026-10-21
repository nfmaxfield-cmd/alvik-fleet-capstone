"""Stage 2 training extras: rejected guesses as "not an object" examples, copy-paste augmentation
(and the with/without comparison), and the report by situation.

Fast tests run always; the ones that train a model on synthetic scenes take a minute or two.
"""
import csv
import json
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REC = os.path.dirname(HERE)
TOOLS = os.path.dirname(REC)
DET = os.path.join(TOOLS, "detector")
for p in (REC, os.path.join(TOOLS, "label"), DET, os.path.join(DET, "tests"), HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import detcore  # noqa: E402
import project as projmod  # noqa: E402
import situations  # noqa: E402
import trainer  # noqa: E402
from test_studio import fake_recording, write_labels  # noqa: E402


# --------------------------------------------------------------------------------------------
# report by situation: counts on a hand-built example
# --------------------------------------------------------------------------------------------

def _hand_built():
    """200 x 100 picture; robot size 20, hand size 40.
    frames 0-8 (every 0.5 s): robot S alone at (100, 60), not moving;
    frame 10 (t = 5 s): A(50,50) and B(70,50) close together; C(150,50) with a hand H(170,50) beside it;
    D(8,50) alone at the picture's edge."""
    frames, times = {}, {}
    for f in range(9):
        frames[f] = [dict(cls="alvik", x=100.0, y=60.0, obj="S")]
        times[f] = f * 0.5
    frames[10] = [dict(cls="alvik", x=50.0, y=50.0, obj="A"), dict(cls="alvik", x=70.0, y=50.0, obj="B"),
                  dict(cls="alvik", x=150.0, y=50.0, obj="C"), dict(cls="hand", x=170.0, y=50.0, obj="H"),
                  dict(cls="alvik", x=8.0, y=50.0, obj="D")]
    times[10] = 5.0
    return frames, times


def test_situation_tags_on_a_hand_built_example():
    frames, times = _hand_built()
    tg = situations.tag_frames(frames, {"alvik": 20, "hand": 40}, ["hand"], (200, 100), None, times)
    assert [sorted(tg[(f, 0)]) for f in range(4)] == [["alone"]] * 4            # not yet 2 s of history
    assert [sorted(tg[(f, 0)]) for f in range(4, 9)] == [["alone", "still"]] * 5
    assert sorted(tg[(10, 0)]) == ["close"] and sorted(tg[(10, 1)]) == ["close"]       # A and B
    assert sorted(tg[(10, 2)]) == ["person"]                                           # C beside the hand
    assert sorted(tg[(10, 3)]) == ["close", "edge"]                                    # the hand: next to C, 30 px from the edge
    assert sorted(tg[(10, 4)]) == ["alone", "edge"]                                    # D


def test_situation_counts_and_flags():
    frames, times = _hand_built()
    tg = situations.tag_frames(frames, {"alvik": 20, "hand": 40}, ["hand"], (200, 100), None, times)
    samples, tags = [], {"blk": {}}
    for f, objs in frames.items():
        samples.append(dict(recording="blk", frame=f, objects=[dict(o, heading=None) for o in objs]))
        for i, o in enumerate(objs):
            tags["blk"][situations.label_key(f, o["cls"], o["x"], o["y"])] = sorted(tg[(f, i)])
    # the model finds S in frames 4-8, C and the hand, and nothing else
    found = {(4, 0), (5, 0), (6, 0), (7, 0), (8, 0), (10, 2), (10, 3)}
    preds = {i: [dict(cls=o["cls"], x=o["x"] + 1, y=o["y"], heading_deg=None, score=0.9)
                 for k, o in enumerate(s["objects"]) if (s["frame"], k) in found] for i, s in enumerate(samples)}
    rows = {r["situation"]: r for r in situations.score(samples, preds, tags, ["alvik", "hand"], 8.0)}
    assert (rows["alone"]["n_labels"], rows["alone"]["tp"]) == (10, 5)       # S x9 and D; found S in 4-8
    assert (rows["close"]["n_labels"], rows["close"]["tp"]) == (3, 1)        # A, B, hand; found the hand
    assert (rows["person"]["n_labels"], rows["person"]["tp"]) == (1, 1)      # C
    assert (rows["edge"]["n_labels"], rows["edge"]["tp"]) == (2, 1)          # D, hand
    assert (rows["still"]["n_labels"], rows["still"]["tp"]) == (5, 5)
    assert (rows["all"]["n_labels"], rows["all"]["tp"]) == (14, 7)
    assert rows["alone"]["n_objects"] == 2 and rows["still"]["n_objects"] == 1      # S and D; S
    assert rows["still"]["found"] == 1.0 and rows["alone"]["found"] == 0.5
    assert all(r["few"] for r in rows.values())            # every group is tiny: flagged
    assert abs(rows["alone"]["centre_err_px_mean"] - 1.0) < 1e-6


def test_edge_of_the_work_area():
    frames = {0: [dict(cls="alvik", x=60.0, y=50.0, obj="a"), dict(cls="alvik", x=100.0, y=50.0, obj="b")]}
    area = [[50, 10], [150, 10], [150, 90], [50, 90]]
    tg = situations.tag_frames(frames, {"alvik": 20}, [], (200, 100), area, {0: 0.0})
    assert "edge" in tg[(0, 0)] and "edge" not in tg[(0, 1)]          # 10 px inside the corner vs. the middle


def test_not_still_when_it_moves_or_labels_have_gaps():
    mk = lambda xs: {i: [dict(cls="alvik", x=x, y=50.0, obj="1")] for i, x in enumerate(xs)}      # noqa: E731
    t = {i: i * 0.5 for i in range(9)}
    moving = situations.tag_frames(mk([100, 100, 100, 100, 100, 100, 100, 100, 112]), {"alvik": 20}, [], (400, 100), None, t)
    assert "still" not in moving[(8, 0)] and "still" in moving[(7, 0)]
    sparse = {0: [dict(cls="alvik", x=100.0, y=50.0, obj="1")], 20: [dict(cls="alvik", x=100.0, y=50.0, obj="1")]}
    assert "still" not in situations.tag_frames(sparse, {"alvik": 20}, [], (400, 100), None, {0: 0.0, 20: 10.0})[(20, 0)]


# --------------------------------------------------------------------------------------------
# a project: situation tags are written for the held-back blocks only
# --------------------------------------------------------------------------------------------

def _project_with_labels(tmp_path, every=7, extra=None):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    proj.set_classes([dict(name="alvik", has_front=True, size_px=30.0), dict(name="hand", height_varies=True, size_px=60.0)])
    proj.data["settings"]["block_s"] = 5
    fake_recording(str(tmp_path / "r"), n=600)
    rid = proj.add_recording(str(tmp_path / "r"))[0]
    rows = []
    for fr in range(0, 600, every):
        rows.append([fr, "", "1", "alvik", 100, 100, "", 0, "user", "confirmed", ""])
        if fr % 21 == 0:
            rows.append([fr, "", "2", "hand", 120, 100, "", 0, "user", "confirmed", ""])
    rows += extra or []
    write_labels(proj.labels_dir(rid), rows)
    return proj, rid


def test_situation_tags_cover_test_blocks_only(tmp_path):
    proj, rid = _project_with_labels(tmp_path)
    assign, info = trainer.build_dataset(proj, str(tmp_path / "d"), block_s=5)
    tr = trainer.Trainer(proj, log=lambda *a: None)
    out = str(tmp_path / "tags.json")
    tags = trainer.write_situation_tags(tr, str(tmp_path / "d"), assign, info, out)
    test_blocks = {n for n, p in assign.items() if p == "test"}
    assert test_blocks and set(tags) == test_blocks
    saved = json.load(open(out))["tags"]
    assert saved == {k: v for k, v in tags.items()}
    some = next(iter(tags[next(iter(test_blocks))].items()))
    assert "|alvik|100.00|100.00" in some[0] or "|hand|120.00|100.00" in some[0]
    flat = [t for b in tags.values() for v in b.values() for t in v]
    assert "person" in flat and "still" in flat       # the alvik stands beside a hand and never moves


def test_negative_counts_and_default_unchanged(tmp_path):
    rej = [[3, "", "", "alvik", 300, 300, "", 0, "motion", "rejected", "0.4"],
           [4, "", "", "alvik", 310, 310, "", 0, "motion", "rejected", "0.4"]]
    proj, rid = _project_with_labels(tmp_path, extra=rej)
    assign, info = trainer.build_dataset(proj, str(tmp_path / "d"), block_s=5)
    n = trainer.negative_counts(str(tmp_path / "d"), proj.class_names(), assign)
    n_all = sum(1 for part in assign.values() if part == "train")
    assert n_all > 0 and n["spots"] <= 2 and n["frames_only"] == n["frames"]    # frames 3 and 4 hold only rejected rows
    proj2, _ = _project_with_labels(tmp_path / "plain")
    a2, _ = trainer.build_dataset(proj2, str(tmp_path / "d2"), block_s=5)
    assert trainer.negative_counts(str(tmp_path / "d2"), proj2.class_names(), a2) == dict(spots=0, frames=0, frames_only=0)


# --------------------------------------------------------------------------------------------
# copy-paste setting, the verdict and the report
# --------------------------------------------------------------------------------------------

def test_paste_setting_defaults_off(tmp_path):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    assert trainer.paste_mode(proj) == "off"
    for v, want in ((True, "on"), ("on", "on"), ("compare", "compare"), ("off", "off"), (False, "off"), (None, "off")):
        proj.data["settings"]["copy_paste"] = v
        assert trainer.paste_mode(proj) == want


def test_paste_verdict_is_plain_and_needs_a_clear_gain():
    a = dict(precision=0.80, recall=0.70, f1=0.747)
    better = trainer.decide_paste(a, dict(precision=0.85, recall=0.80, f1=0.824))
    same = trainer.decide_paste(a, dict(precision=0.80, recall=0.71, f1=0.752))
    worse = trainer.decide_paste(a, dict(precision=0.70, recall=0.70, f1=0.70))
    assert (better["used"], better["winner"]) == (True, "with") and "better" in better["text"]
    assert (same["used"], same["winner"]) == (False, "without") and "no clear difference" in same["text"]
    assert (worse["used"], worse["winner"]) == (False, "without") and "worse" in worse["text"]
    assert "82%" in better["text"] and "75%" in better["text"]


def test_report_shows_situations_and_both_copy_paste_scores(tmp_path):
    import report
    proj, rid = _project_with_labels(tmp_path)
    mdir = proj.model_dir("v1")
    os.makedirs(mdir, exist_ok=True)
    pt = lambda m, f: dict(model=m, label="Model v1" if m == "v1" else "No model (motion)", n_train_frames=0 if m == "none" else 40,   # noqa: E731
                           precision=f, recall=f, f1=f, centre_err_px=2.0, heading_err_deg=5.0, ms_per_frame=9.0, n_labels=100, n_objects=8)
    projmod.write_json(os.path.join(proj.folder, "models", "curve.json"),
                       dict(updated="now", points=[pt("none", 0.4), pt("v1", 0.9)], test=dict(frames=50, blocks=3, block_s=5, note="n")))
    row = lambda k, n, found, few=False: dict(situation=k, name=situations.NAMES.get(k, "All labels"), n_labels=n, n_objects=9,   # noqa: E731
                                              tp=int(n * found), found=found, centre_err_px=2.0, heading_err_deg=4.0, few=few)
    mk = lambda f: [row("alone", 120, f), row("close", 40, f - 0.3), row("person", 12, f - 0.5, True), row("edge", 60, f - 0.1),   # noqa: E731
                    row("still", 70, f), row("all", 300, f)]
    projmod.write_json(os.path.join(proj.folder, "models", "situations.json"),
                       dict(updated="now", test={}, models={"none": mk(0.5), "v1": mk(0.9)}))
    with proj.lock:
        proj.data["models"].append(dict(id="v1", created="2026-10-20T10:00:00", status="ready", classes=proj.class_names(), metrics={},
                                        hard_negatives=dict(spots=14, frames=9, frames_only=5),
                                        copy_paste=dict(mode="compare", compared=True, used=False, minutes_each=3.0,
                                                        without=dict(precision=.9, recall=.9, f1=.9, centre_err_px=2.0, n_labels=100),
                                                        **{"with": dict(precision=.8, recall=.8, f1=.8, centre_err_px=2.5, n_labels=100)},
                                                        text="With copy-paste ... Copy-paste is worse here, so this model was trained without it.")))
        proj.save()
    html = report.html_report(proj)
    assert "By situation (model v1)" in html and "Close together" in html and "Near a person" in html
    assert "Too few to trust" in html                       # the 12-label group is flagged
    assert "finds the fewest where objects are <b>close together</b>" in html        # 60%: the weakest group with enough labels
    assert "What training used" in html and "14 guesses you rejected" in html
    assert "Without copy-paste (kept)" in html and "With copy-paste</td>" in html and "worse here" in html


# --------------------------------------------------------------------------------------------
# compare mode: train with and without copy-paste, keep the better
# --------------------------------------------------------------------------------------------

@pytest.mark.parametrize("with_f1,winner", [(0.90, "with"), (0.70, "without"), (0.801, "without")])
def test_compare_mode_trains_both_and_keeps_the_better(tmp_path, with_f1, winner):
    proj = projmod.Project(str(tmp_path / "p"), create_name="P")
    tr = trainer.Trainer(proj, log=lambda *a: None)
    tr.job = dict(message="")
    mdir = str(tmp_path / "m")
    run = os.path.join(mdir, "run")
    calls = []

    def fake_call(cmd, logf, timeout, progress=None):
        calls.append((list(cmd), dict(progress or {})))
        out = cmd[cmd.index("--out") + 1]
        if cmd[1].endswith("train.py"):
            os.makedirs(out, exist_ok=True)
            for f in ("best.pt", "last.pt"):
                with open(os.path.join(out, f), "w") as fh:
                    fh.write("with" if "--paste" in cmd else "without")
            with open(os.path.join(out, "train_log.csv"), "w") as fh:
                fh.write("epoch,val_f1\n1,0.5\n")
        else:                                           # export.py: the onnx records which training made it
            with open(out, "w") as fh:
                fh.write(open(cmd[2]).read())
    tr._call = fake_call
    tr._score_variant = lambda data, split, onnx, out, logf, python: (
        dict(precision=with_f1, recall=with_f1, f1=with_f1, centre_err_px=2.0, n_labels=50.0, n_objects=6.0)
        if open(onnx).read() == "with" else dict(precision=0.8, recall=0.8, f1=0.8, centre_err_px=2.0, n_labels=50.0, n_objects=6.0))
    cmd = ["python", "train.py", "--labels", "d", "--out", run, "--max-minutes", "3.0", "--init-from", "prev.pt"]
    cp = tr._train_compare("v1", 3.0, "python", {}, mdir, None, cmd, run, str(tmp_path / "areas.json"), "d", "s.json")
    trains = [c for c, _ in calls if c[1].endswith("train.py")]
    assert len(trains) == 2 and "--paste" not in trains[0] and "--paste" in trains[1]
    assert all(c[c.index("--max-minutes") + 1] == "1.5" and c[c.index("--init-from") + 1] == "prev.pt" for c in trains)
    assert trains[0][trains[0].index("--out") + 1] != trains[1][trains[1].index("--out") + 1]
    assert "without copy-paste, 1 of 2" in calls[0][1]["tag"]
    assert cp["winner"] == winner and cp["used"] == (winner == "with") and cp["compared"]
    assert open(os.path.join(run, "best.pt")).read() == winner                       # the better model is the one kept
    assert cp["without"]["f1"] == 0.8 and cp["with"]["f1"] == with_f1
    assert cp["text"].count("%") >= 6 and ("so this model uses it" in cp["text"]) == (winner == "with")
    left = [f for k in ("with", "without") for f in os.listdir(os.path.join(mdir, "compare", k))]
    assert "best.pt" not in left and "model.onnx" not in left and "train_log.csv" in left


@pytest.mark.skipif(os.environ.get("RECOGNITION_TRAIN") != "1", reason="slow: set RECOGNITION_TRAIN=1")
def test_compare_mode_through_the_studio(tmp_path):
    """The real thing: a 60 s synthetic recording, copy-paste set to compare, rejected guesses present."""
    import subprocess
    import time

    import studio as studiomod
    rec = str(tmp_path / "rec60")
    subprocess.run([sys.executable, os.path.join(TOOLS, "label", "tests", "synth.py"), rec, "--seconds", "60"],
                   check=True, capture_output=True)
    proj = projmod.Project(str(tmp_path / "p"), create_name="Compare")
    proj.finish_setup(classes=projmod.PRESETS[0]["classes"])
    proj.data["settings"]["block_s"] = 10
    proj.data["settings"]["copy_paste"] = "compare"
    proj.save()
    rid = proj.add_recording(rec, "overhead")[0]
    st = studiomod.Studio(proj, log=lambda *a: None)
    st.open(rid, "none", wait=True)
    s = st.session
    truth = {}
    with open(os.path.join(rec, "truth.csv")) as f:
        for r in csv.DictReader(f):
            truth.setdefault(int(r["frame"]), []).append(r)
    for f in s.selected:
        objs = [dict(obj=None, cls=r["cls"] if r["cls"] in s.classes else "unknown", x=float(r["x"]), y=float(r["y"]),
                     heading_deg=float(r["heading_deg"]) if r["cls"] == "alvik" else None,
                     heading_known=r["cls"] == "alvik", source="user", status="corrected")
                for r in truth.get(f, []) if 0 <= float(r["x"]) <= 640 and 0 <= float(r["y"]) <= 480]
        objs.append(dict(obj=None, cls="alvik", x=5.0, y=5.0, source="motion", status="rejected"))      # a rejected guess
        s.save_frame(f, objs, "reviewed", "user")
    mid = st.trainer.start(minutes=2.0)
    while st.trainer.busy():
        time.sleep(3)
    assert st.trainer.job["status"] == "done", st.trainer.job["message"]
    m = proj.model(mid)
    cp = m["copy_paste"]
    assert cp["compared"] and cp["without"]["f1"] is not None and cp["with"]["f1"] is not None
    assert m["hard_negatives"]["spots"] > 0
    sit = json.load(open(os.path.join(proj.folder, "models", "situations.json")))
    assert {"none", mid} <= set(sit["models"]) and sit["models"][mid][-1]["situation"] == "all"
    import report
    html = report.html_report(proj)
    assert "By situation" in html and "What training used" in html and ("Copy-paste" in html)
    print("COMPARE:", cp["text"])
    print("SITUATIONS:", [(r["situation"], r["n_labels"], r["found"]) for r in sit["models"][mid]])
