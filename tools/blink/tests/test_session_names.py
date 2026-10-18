"""Code -> name from a console session's PARAM blink_code (blink.py --session, check_session.py)."""
import os
import subprocess
import sys

import blink

TOOLS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, TOOLS)
import check_session as cs  # noqa: E402


def session(tmp_path, robots):
    """robots: [(name, run, blink_code, blink_codes)] -> a console folder with param.csv."""
    d = tmp_path / "session"
    d.mkdir(exist_ok=True)
    lines = ["robot,t_recv,run,name,value"]
    for name, run, code, on in robots:
        lines += [f"{name},1000,{run},blink_codes,{on:.4f}", f"{name},1000,{run},blink_code,{code:.4f}",
                  f"{name},1000,{run},sync_ms,300.0000"]
    (d / "param.csv").write_text("\n".join(lines) + "\n")
    return str(d)


def test_code_for_matches_firmware():
    assert [cs.blink_code_for(n, 0) for n in ("Left", "Center", "Right", "left", "Bob", "LEFT")] == [1, 2, 3, 4, 4, 4]
    assert cs.blink_code_for("Bob", "2.0000") == 2 and cs.blink_code_for("Left", 3) == 3
    assert cs.blink_code_for("Left", 7) == 1 and cs.blink_code_for("Left", -1) == 1     # outside 1..4: from the name
    assert cs.blink_code_for("Right", "") == 3


def test_default_session_gives_default_names(tmp_path):
    s = session(tmp_path, [("Left", "a-1", 0, 1), ("Center", "b-1", 0, 1), ("Right", "c-1", 0, 1)])
    log = []
    assert blink.names_from_session(s, log=log.append) == blink.DEFAULT_NAMES
    assert not any(l.startswith("warning") for l in log)
    codes, notes = cs.session_blink_codes(s)
    assert codes == {"Left": {1: ["a-1"]}, "Center": {2: ["b-1"]}, "Right": {3: ["c-1"]}} and notes == []


def test_blink_code_param_and_odd_names(tmp_path):
    # Bob chose code 2 by PARAM; "left" (lower case) gets the 'other' code 4; nobody blinks 1 or 3
    s = session(tmp_path, [("Bob", "a-1", 2, 1), ("left", "b-1", 0, 1), ("Right", "c-1", 0, 0)])   # Right: lights off
    log = []
    names = blink.names_from_session(s, log=log.append)
    assert names == {1: "Left", 2: "Bob", 3: "Right", 4: "left"}
    w = " ".join(log)
    assert "blink_code = 2" in w and "'left'" in w and "'Left'" in w


def test_shared_code_warned_and_names_unique(tmp_path):
    s = session(tmp_path, [("Bob", "a-1", 0, 1), ("Eve", "b-1", 0, 1), ("Left", "c-1", 2, 1)])
    log = []
    names = blink.names_from_session(s, log=log.append)
    assert names[4] == "Bob/Eve" and names[2] == "Left" and names[1] == "code1"   # Left's name taken by code 2
    assert len(set(names.values())) == 4
    assert any("both blink code 4" in l for l in log) and any("both blink code" not in l and "blink_code = 2" in l
                                                              for l in log)


def test_check_session_warns(tmp_path):
    s = session(tmp_path, [("Left", "a-1", 0, 1), ("Left", "a-2", 3, 1), ("Right", "c-1", 0, 1)])
    r = subprocess.run([sys.executable, os.path.join(TOOLS, "check_session.py"), s], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    out = r.stdout
    assert "BLINK CODES" in out
    assert "blink: Left: PARAM blink_code = 3" in out
    assert "Left and Right both blink code 3" in out and "different codes in different runs" in out
