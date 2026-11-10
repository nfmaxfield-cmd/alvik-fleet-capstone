"""(11-10a) A camera reader orphaned by an earlier run holds its UDP port: stale_bridge.clear stops it; a program that
is not one of our readers, holding the port, is left alone."""
import os
import socket
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import stale_bridge as SB  # noqa: E402

HOLD = "import socket,time\ns=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)\ns.bind(('127.0.0.1',%d))\ntime.sleep(60)\n"


def _free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _orphan(name, port):
    d = tempfile.mkdtemp()
    f = os.path.join(d, name)
    with open(f, "w") as fh:
        fh.write(HOLD % port)
    # the shell exits at once: the holder is re-parented (orphaned), as when Mission Control was restarted mid-run
    subprocess.run(["bash", "-c", "nohup %s %s >/dev/null 2>&1 &" % (sys.executable, f)], check=True)
    for _ in range(50):
        h = SB.holders(port)
        if h:
            return h[0]
        time.sleep(0.1)
    raise AssertionError("holder did not start")


def test_orphaned_reader_is_stopped():
    port = _free_port()
    pid = _orphan("tag_bridge.py", port)
    said = []
    done = SB.clear((port,), log=said.append, wait_s=3.0)
    assert (pid, "stopped") in done, (done, said)
    assert not SB.holders(port)
    assert any("LEFTOVER" in s for s in said)


def test_other_program_left_alone():
    port = _free_port()
    pid = _orphan("something_else.py", port)
    try:
        said = []
        done = SB.clear((port,), log=said.append, wait_s=1.0)
        assert done == [(pid, "left alone (not a camera reader)")]
        assert SB.holders(port) == [pid]
    finally:
        os.kill(pid, 9)


def test_switch_off(monkeypatch):
    monkeypatch.setenv("MIRLT_STALE_BRIDGE", "0")
    assert SB.clear((5011,)) == []
