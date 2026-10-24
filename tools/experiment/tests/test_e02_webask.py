"""The Pi run's questions on the live page: a question shows in status.json and an answer posted to /answer is
returned (no robots, no camera)."""
import json
import os
import sys
import threading
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import e02_view  # noqa: E402


class _R:
    phase, order, tracker, feed, st, cam, board = "setup", "", None, None, {}, None, None


def test_question_and_answer_over_http():
    v = e02_view.LiveView(_R(), port=18093, bind="127.0.0.1")
    v.health = lambda: None
    v.start()
    box = []
    th = threading.Thread(target=lambda: box.append(v.ask("Carry on? (y/n)", "y", timeout_s=10)))
    th.start()
    q = None
    for _ in range(50):
        s = json.loads(urllib.request.urlopen("http://127.0.0.1:18093/status.json", timeout=2).read())
        q = s.get("question")
        if q:
            break
        time.sleep(0.1)
    assert q and q["prompt"] == "Carry on? (y/n)" and q["default"] == "y"
    req = urllib.request.Request("http://127.0.0.1:18093/answer", data=json.dumps(dict(id=q["id"], text="n")).encode(),
                                 method="POST", headers={"Content-Type": "application/json"})
    urllib.request.urlopen(req, timeout=2).read()
    th.join(5)
    assert box == ["n"]
    s = json.loads(urllib.request.urlopen("http://127.0.0.1:18093/status.json", timeout=2).read())
    assert s.get("question") is None
    v.stop()


def test_no_answer_times_out():
    v = e02_view.LiveView(_R(), port=0)
    assert v.ask("Anything?", "", timeout_s=0.5) is None
