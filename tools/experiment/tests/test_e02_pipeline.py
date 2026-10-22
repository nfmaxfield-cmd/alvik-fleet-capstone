"""The whole E02 pipeline, as after a real session: the runner's simulator dry run (with the simulated overhead camera
recorded like camrec.py: E02_SIM_RECORD=1), then the one analysis command on its session folder:

    E02_SLOW=1 python -m pytest -q tools/experiment/tests/test_e02_pipeline.py        # dry run + analysis (~20 min)
    E02_PIPELINE_SESSION=logs/e02-<...>-dryrun python -m pytest -q tools/experiment/tests/test_e02_pipeline.py
                                                                     # analysis only, on a recorded dry run (~2 min)

Checks: the command finds every input by itself, every run of the runner's list gets a row, the robot's FIND/BUMP
rows join, the analysis's camera poses agree with the simulator's truth, and report.md keeps the house rules."""
import csv
import glob
import math
import os
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(EXP))

SESSION = os.environ.get("E02_PIPELINE_SESSION")
pytestmark = pytest.mark.skipif(not (SESSION or os.environ.get("E02_SLOW")),
                                reason="slow: set E02_SLOW=1 (dry run ~20 min) or E02_PIPELINE_SESSION=<recorded dry run>")

DRY = ["--dry-run", "--dry-true-calib", "--no-side", "--view-port", "0", "--naming-s", "10", "--blocks", "2",
       "--routes", "1,2,3", "--max-runs", "10", "--seed", "21"]


def _csv(p):
    with open(p, newline="") as f:
        return list(csv.DictReader(f))


@pytest.fixture(scope="module")
def analysed():
    sdir = SESSION
    if not sdir:
        before = set(glob.glob(os.path.join(REPO, "logs", "e02-*-dryrun")))
        env = dict(os.environ, E02_SIM_RECORD="1")
        subprocess.run([sys.executable, os.path.join(EXP, "e02.py")] + DRY, cwd=REPO, env=env, timeout=3600,
                       stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        new = sorted(set(glob.glob(os.path.join(REPO, "logs", "e02-*-dryrun"))) - before)
        assert new, "the dry run made no session folder"
        sdir = new[-1]
    sdir = os.path.abspath(sdir)
    assert os.path.exists(os.path.join(sdir, "sim", "recording", "cam0", "video.avi")), "dry run not recorded"
    out = os.path.join(sdir, "analysis")
    p = subprocess.run([sys.executable, os.path.join(EXP, "e02_analysis.py"), "--session", sdir], cwd=REPO,
                       capture_output=True, text=True, timeout=1800)
    assert p.returncode == 0, p.stdout[-2000:] + p.stderr[-3000:]
    return sdir, out, p.stdout


def test_one_command_writes_every_output(analysed):
    _, out, _ = analysed
    for f in ("report.md", "runs.csv", "summary.csv", "comparisons.csv", "bumps.csv"):
        assert os.path.getsize(os.path.join(out, f)) > 0, f


def test_every_run_in_the_runners_list_has_a_row_and_most_are_measured(analysed):
    sdir, out, _ = analysed
    listed = _csv(os.path.join(sdir, "e02_runs.csv"))
    rows = _csv(os.path.join(out, "runs.csv"))
    assert len(rows) == len(listed) and len({r["run_id"] for r in rows}) == len(rows)
    done = [r for r in rows if r["status"] == "done"]
    measured = [r for r in done if r["end_err_cm"] not in ("", "nan")]
    assert done and len(measured) >= 0.8 * len(done), [(r["run_id"], r["flags"]) for r in done]
    # FIND rows (keyed by the find's own run id) joined: the believed start is compared with the camera
    assert sum(1 for r in measured if r.get("find_err_cm") not in ("", None)) >= 0.8 * len(measured)
    assert sum(1 for r in measured if r.get("far_intent_from") == "E02 far_goal") >= 0.8 * len(measured)


def test_camera_poses_agree_with_the_simulators_truth(analysed):
    """Start and end measured by the analysis against the simulator's true pose at those moments (truth.csv is on
    the laptop clock; the run list's times are on the camera's: laptop = camera + pi_offset_s)."""
    import json
    sdir, out, _ = analysed
    po = json.load(open(os.path.join(sdir, "e02_state.json")))["setup"]["pi_offset_s"]
    truth = {}
    for r in _csv(os.path.join(sdir, "sim", "truth.csv")):
        truth.setdefault(r["robot"], []).append((float(r["t"]), float(r["x_cm"]), float(r["y_cm"])))
    listed = {r["run_id"]: r for r in _csv(os.path.join(sdir, "e02_runs.csv"))}
    errs = []
    for r in _csv(os.path.join(out, "runs.csv")):
        if r.get("end_x") in ("", None) or r["run_id"] not in listed:
            continue
        L = listed[r["run_id"]]
        tr = np.array(truth[r["robot"]])
        for stage, t in (("start", float(L["t_start"]) - 0.3), ("end", float(L["t_end"]) + 0.5)):
            tx, ty = np.interp(t + po, tr[:, 0], tr[:, 1]), np.interp(t + po, tr[:, 0], tr[:, 2])
            errs.append(math.hypot(float(r[stage + "_x"]) - tx, float(r[stage + "_y"]) - ty))
    assert len(errs) >= 6
    assert np.median(errs) < 1.0 and np.percentile(errs, 90) < 2.0, np.round(sorted(errs), 2)


def test_the_report_keeps_the_house_rules(analysed):
    _, out, _ = analysed
    text = open(os.path.join(out, "report.md")).read()
    heads = [ln for ln in text.splitlines() if ln.startswith("## ")]
    assert heads == ["## Premise", "## Data", "## Conclusion", "## What's next", "## Two sweeps"]
    assert text.index("New tests that may now be needed") < text.index("Possible inaccuracies in what this relied on")
    for img in [ln for ln in text.splitlines() if ln.startswith("![")]:
        path = img[img.index("(") + 1:-1]
        assert os.path.exists(os.path.join(out, path)), path
        after = text[text.index(img) + len(img):].lstrip("\n").splitlines()[0]
        assert after.startswith("*") and after.count(". ") == 0, after            # one plain sentence under each chart
    if "| A vs B |" in text:
        first_table = text.index("| A vs B |")
        assert text.index("Welch's t-test:") > first_table
        assert text.index("Welch's t-test:") < text.index("| A vs B |", first_table + 1) \
            if text.count("| A vs B |") > 1 else True
    assert text.count("Welch's t-test:") <= 1 and "how to read" not in text.lower()
