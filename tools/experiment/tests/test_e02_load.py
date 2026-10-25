"""Planned load and the speed watch (e02_load.py)."""
import os
import sys
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import e02_load as LD  # noqa: E402


def _row(t, fps, age):
    return dict(t=t, fps_handled=fps, age_mean_s=age)


def test_parse_kinds():
    assert LD.parse("cpu") == ("cpu", 1) and LD.parse("cpu:2") == ("cpu", 2)
    assert LD.parse("disk:/tmp") == ("disk", "/tmp")
    assert LD.parse("cmd:sleep 5") == ("cmd", "sleep 5")
    for bad in ("", "gpu", "cpu:0", "disk:", "cmd:"):
        with pytest.raises(ValueError):
            LD.parse(bad)


def test_speed_watch_learns_a_baseline_then_flags_a_slow_span_and_its_end():
    w = LD.SpeedWatch(base_rows=10, hold_s=3)
    t = 0
    for _ in range(10):
        assert w.feed(_row(t, 20.0, 0.11)) is None
        t += 1
    assert w.base_fps == 20.0 and abs(w.base_age - 0.11) < 1e-9
    assert w.feed(_row(t, 12.0, 0.15), load_on=True) is None           # one slow second: a hiccup
    assert w.feed(_row(t + 1, 19.0, 0.11)) is None
    out = [w.feed(_row(t + 2 + k, 12.0, 0.16), load_on=True) for k in range(3)]
    assert out == [None, None, "slow"] and w.slow
    back = [w.feed(_row(t + 5 + k, 20.0, 0.11)) for k in range(3)]
    assert back == [None, None, "normal"] and not w.slow
    sp = w.spans[0]
    assert sp["fps_min"] == 12.0 and sp["load"] and sp["t1"] is not None


def test_baseline_skips_seconds_with_the_load_on():
    w = LD.SpeedWatch(base_rows=3, hold_s=2)
    w.feed(_row(0, 5.0, 1.0), load_on=True)
    for k in range(3):
        w.feed(_row(1 + k, 20.0, 0.1))
    assert w.base_fps == 20.0


def test_cmd_load_switches_on_and_off_and_records_each_switch():
    rec, marks = [], []
    ld = LD.Load("cmd:sleep 30", on_s=0.5, off_s=0.5, log=lambda m: None, mark=marks.append, record=rec.append)
    ld.start()
    time.sleep(1.3)
    ld.stop()
    assert not ld.on and ld._cmd is None
    assert [r["on"] for r in rec][:3] == [True, False, True] and rec[-1]["on"] is False
    assert marks[0] == "e02 load ON: cmd:sleep 30"


def test_cpu_load_starts_and_stops_its_processes():
    ld = LD.Load("cpu:1", on_s=5, off_s=5, log=lambda m: None)
    ld._switch(True)
    assert ld.on and all(p.is_alive() for p in ld._procs)
    procs = list(ld._procs)
    ld._switch(False)
    assert not ld.on and not any(p.is_alive() for p in procs)
