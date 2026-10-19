"""tools/edge_check.py on synthetic E0c edge rows (sheet + RAW logs).

    python -m pytest tools/tests/test_edge_check.py -q
"""
import csv
import random

import edge_check as ec

ROBOTS = ("Left", "Center", "Right")
LEVELS = ("still60", "tof40", "edge_n15", "edge_n10", "edge_n5", "edge_in", "edge_over", "edge_tape")


def make(tmp_path, line_level, dbot_level, seed=0):
    """line_level(robot, level, k) -> mean reading of sensor k; dbot_level(robot, level) -> mean dbot."""
    rng = random.Random(seed)
    data = tmp_path / "data"
    logs = data / "logs" / "s1"
    logs.mkdir(parents=True)
    sheet, raw = [], []
    n = 0
    for rb in ROBOTS:
        for lv in LEVELS:
            n += 1
            rid = f"AB{n:02d}-{n}"
            sheet.append(dict(order=n, robot=rb, level=lv, run_id=rid, excluded=""))
            for i in range(200):                       # 4 s at 50 per second
                row = dict(run=rid, ms=1000 + 20 * i, lifted=0,
                           dbot_cm=round(rng.gauss(dbot_level(rb, lv), 0.3), 1))
                for k, c in enumerate(ec.SENSORS):
                    row[c] = int(rng.gauss(line_level(rb, lv, k), 12))
                raw.append(row)
    with open(data / "measurements.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(sheet[0]))
        w.writeheader()
        w.writerows(sheet)
    with open(logs / "raw.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(raw[0]))
        w.writeheader()
        w.writerows(raw)
    return str(tmp_path)


OFFSET = {"Left": 0, "Center": 40, "Right": -30}      # robots differ; the learned level cancels it


def ply_bot(rb, lv):
    return {"edge_n5": 15.5, "edge_in": 15.8, "edge_over": 16.0}.get(lv, 9.0) + {"Left": 0, "Center": 0.4, "Right": -0.3}[rb]


def test_separable_edge_reads_higher(tmp_path, capsys):
    def line(rb, lv, k):
        base = 150 + OFFSET[rb] + 10 * k
        return base + {"edge_over": 850, "edge_tape": 110}.get(lv, 0)
    data = ec.load(make(tmp_path, line, ply_bot))
    res = ec.line_cue(data)
    assert res["ok"] and res["sign"] == 1 and res["mx"] == 0
    assert 110 + 50 < res["thr"] < 850 - 50                  # clear of the tape and of the edge
    bot = ec.bottom_cue(data)
    assert bot["ok"] and bot["warn_cm"] == 5.0                 # n10 still plywood-like, n5 jumped
    assert 9.5 < bot["thr"] < 15
    mode, cmds = ec.recommend(res, bot)
    assert mode == 3 and "set edge_dir 1" in cmds
    assert ec.main([make(tmp_path / "again", line, ply_bot)]) == 0
    assert "VERDICT: both cues" in capsys.readouterr().out


def test_edge_reads_lower(tmp_path):
    def line(rb, lv, k):
        return 500 + OFFSET[rb] + {"edge_over": -300, "edge_tape": 200}.get(lv, 0)
    res = ec.line_cue(ec.load(make(tmp_path, line, ply_bot)))
    assert res["ok"] and res["sign"] == -1


def test_tape_beyond_edge_needs_window(tmp_path):
    def line(rb, lv, k):
        return 150 + OFFSET[rb] + {"edge_over": 400, "edge_tape": 750}.get(lv, 0)
    res = ec.line_cue(ec.load(make(tmp_path, line, ply_bot)))
    assert res["ok"] and res.get("window")
    assert 50 < res["thr"] < 400 and 400 < res["mx"] < 750       # change from the plywood level


def test_not_separable(tmp_path):
    def line(rb, lv, k):
        return 150 + OFFSET[rb] + {"edge_over": 600, "edge_tape": 620}.get(lv, 0)   # tape looks like the edge

    def flat_bot(rb, lv):
        return 9.0                                             # bottom zone never sees it
    data = ec.load(make(tmp_path, line, flat_bot))
    line_res, bot_res = ec.line_cue(data), ec.bottom_cue(data)
    assert not line_res["ok"] and not bot_res["ok"]
    assert ec.recommend(line_res, bot_res)[0] == 0


def test_no_runs_yet(tmp_path, capsys):
    (tmp_path / "data").mkdir()
    (tmp_path / "data" / "measurements.csv").write_text("order,robot,level,run_id\n1,Left,edge_in,\n")
    assert ec.main([str(tmp_path)]) == 1
