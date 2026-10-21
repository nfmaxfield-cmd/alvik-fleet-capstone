"""Optional AI helper: stand-in and mocked HTTP only (no real network)."""
import json
import os
import stat
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REC = os.path.dirname(HERE)
if REC not in sys.path:
    sys.path.insert(0, REC)

import ai_helper as ai  # noqa: E402


def pic(v, size=40, noise=0):
    img = np.full((size, size, 3), v, np.uint8)
    if noise:
        img = np.clip(img.astype(int) + np.random.RandomState(1).randint(-noise, noise, img.shape), 0, 255).astype(np.uint8)
    return img


class FakeNet:
    """Records requests; replies from a queue of (status, body) or exceptions."""

    def __init__(self, *replies):
        self.replies, self.calls = list(replies), []

    def __call__(self, url, headers, payload, timeout=0):
        self.calls.append((url, headers, payload))
        r = self.replies.pop(0) if len(self.replies) > 1 else self.replies[0]
        if isinstance(r, Exception):
            raise r
        return r


def anthropic_ok(obj):
    return 200, {"content": [{"type": "text", "text": json.dumps(obj)}], "usage": {"input_tokens": 1000, "output_tokens": 50}}


def openai_ok(obj):
    return 200, {"choices": [{"message": {"content": json.dumps(obj)}}], "usage": {"prompt_tokens": 800, "completion_tokens": 40}}


@pytest.fixture
def home(tmp_path):
    return str(tmp_path / "home")


def make(tmp_path, home, net=None, provider="anthropic", enabled=True, key="sk-test-1234", **cfg):
    h = ai.AIHelper(str(tmp_path / "proj"), user_folder=home, post=net, sleep=lambda s: None)
    h.configure(enabled=enabled, provider=provider, **cfg)
    if key:
        h.set_key(key)
    return h


# --- off by default, key storage ---------------------------------------------------------------------------
def test_off_by_default_sends_nothing(tmp_path, home):
    net = FakeNet(anthropic_ok({}))
    h = ai.AIHelper(str(tmp_path / "proj"), user_folder=home, post=net)
    assert h.is_on() is False and h.status()["indicator"] == "AI helper off"
    with pytest.raises(ai.AIError) as e:
        h.name_object(pic(100))
    assert e.value.kind == "off" and "Nothing was sent" in str(e.value)
    assert net.calls == []


def test_key_is_private_and_never_in_project(tmp_path, home):
    h = make(tmp_path, home)
    p = os.path.join(home, "ai.json")
    assert stat.S_IMODE(os.stat(p).st_mode) == 0o600
    assert stat.S_IMODE(os.stat(home).st_mode) == 0o700
    for r, _, fs in os.walk(str(tmp_path / "proj")):
        for f in fs:
            assert "sk-test-1234" not in open(os.path.join(r, f)).read()
    assert "sk-test-1234" not in json.dumps(h.status())
    h.keys.clear()
    assert h.status()["key_set"] is False


def test_no_key_message(tmp_path, home):
    h = make(tmp_path, home, net=FakeNet(anthropic_ok({})), key="")
    with pytest.raises(ai.AIError) as e:
        h.name_object(pic(100))
    assert e.value.kind == "no_key" and "key" in str(e.value)


# --- Anthropic adapter -------------------------------------------------------------------------------------
def test_anthropic_request_shape_and_answer(tmp_path, home):
    net = FakeNet(anthropic_ok(dict(verdict="object", confidence=0.9, reason="A robot-shaped thing.")))
    h = make(tmp_path, home, net)
    r = h.object_or_background(pic(200), pic(60), strip=[pic(80), pic(90)], known_types=["alvik"])
    url, headers, payload = net.calls[0]
    assert url == "https://api.anthropic.com/v1/messages"
    assert headers["x-api-key"] == "sk-test-1234" and headers["anthropic-version"] == "2023-06-01"
    assert payload["model"] == ai.DEFAULT_ANTHROPIC_MODEL
    blocks = payload["messages"][0]["content"]
    assert sum(b["type"] == "image" for b in blocks) == 4
    assert blocks[1]["type"] == "text" or blocks[0]["type"] == "text"
    assert r["source"] == "ai" and r["verdict"] == "object" and r["confidence"] == 0.9
    log = h.read_log()
    assert len(log) == 1 and log[0]["tokens_in"] == 1000 and log[0]["cost_usd"] > 0
    assert len(log[0]["sent"]["images"]) == 4 and "data" not in json.dumps(log[0])


def test_model_configurable(tmp_path, home):
    net = FakeNet(anthropic_ok(dict(name="cup", confidence=0.8, reason="x")))
    h = make(tmp_path, home, net, model="claude-test-model")
    h.name_object(pic(100))
    assert net.calls[0][2]["model"] == "claude-test-model"


def test_decision_logged(tmp_path, home):
    net = FakeNet(anthropic_ok(dict(name="Cup", confidence=0.8, reason="x")))
    h = make(tmp_path, home, net)
    r = h.name_object(pic(100), known_types=["cup", "box"])
    assert r["name"] == "cup" and r["matches_existing_type"] == "cup"
    assert h.accepted_count() == 0 and h.read_log()[0]["accepted"] is None
    h.record_decision(r["call_id"], True)
    assert h.accepted_count() == 1 and h.read_log()[0]["accepted"] is True
    assert h.status()["calls"] == 1


# --- errors, retries -----------------------------------------------------------------------------------------
@pytest.mark.parametrize("reply,kind,words", [
    ((401, {"error": {"message": "bad"}}), "key", "refused the key"),
    ((404, {}), "model", "does not know the model"),
    ((400, {"error": {"message": "nope"}}), "request", "could not use the request"),
    (ConnectionError("x"), "offline", "No internet connection"),
    (TimeoutError("x"), "timeout", "took too long"),
])
def test_plain_errors(tmp_path, home, reply, kind, words):
    net = FakeNet(reply)
    h = make(tmp_path, home, net)
    with pytest.raises(ai.AIError) as e:
        h.name_object(pic(100))
    assert e.value.kind == kind and words in str(e.value)
    assert h.read_log()[0]["error"] == str(e.value)
    # key problems and bad requests are not retried
    if kind in ("key", "model", "request"):
        assert len(net.calls) == 1
    else:
        assert len(net.calls) == ai.ATTEMPTS


def test_retry_then_success_with_backoff(tmp_path, home):
    net = FakeNet((529, {}), (500, {}), anthropic_ok(dict(name="box", confidence=0.7, reason="x")))
    waits = []
    h = make(tmp_path, home, net)
    h._sleep = waits.append
    assert h.name_object(pic(100))["name"] == "box"
    assert len(net.calls) == 3 and waits == [1.0, 3.0]


def test_busy_gives_up_in_plain_words(tmp_path, home):
    net = FakeNet((429, {}))
    h = make(tmp_path, home, net)
    with pytest.raises(ai.AIError) as e:
        h.name_object(pic(100))
    assert e.value.kind == "busy" and len(net.calls) == ai.ATTEMPTS


def test_unreadable_answer_changes_nothing(tmp_path, home):
    net = FakeNet((200, {"content": [{"type": "text", "text": "I think it is a robot."}], "usage": {}}))
    h = make(tmp_path, home, net)
    with pytest.raises(ai.AIError) as e:
        h.object_or_background(pic(100), pic(100))
    assert e.value.kind == "bad_answer"
    bad = FakeNet(anthropic_ok(dict(verdict="maybe")))
    h2 = make(tmp_path, home, bad)
    with pytest.raises(ai.AIError):
        h2.object_or_background(pic(100), pic(100))


def test_json_in_code_fence_is_read():
    assert ai.extract_json('Sure:\n```json\n{"a": 1}\n```') == {"a": 1}


# --- OpenAI-compatible adapter -----------------------------------------------------------------------------
def test_openai_compatible(tmp_path, home):
    net = FakeNet(openai_ok(dict(verdict="background", confidence=0.7, reason="A stain.")))
    h = make(tmp_path, home, net, provider="openai_compatible", base_url="http://localhost:8000/v1/", model="local-vision")
    r = h.object_or_background(pic(100), pic(100))
    url, headers, payload = net.calls[0]
    assert url == "http://localhost:8000/v1/chat/completions"
    assert headers["Authorization"] == "Bearer sk-test-1234"
    assert payload["model"] == "local-vision" and payload["messages"][0]["role"] == "system"
    assert any(b["type"] == "image_url" and b["image_url"]["url"].startswith("data:image/jpeg;base64,")
               for b in payload["messages"][1]["content"])
    assert r["verdict"] == "background"
    assert h.read_log()[0]["tokens_in"] == 800


def test_openai_odd_answer(tmp_path, home):
    h = make(tmp_path, home, FakeNet((200, {"nope": 1})), provider="openai_compatible")
    with pytest.raises(ai.AIError) as e:
        h.name_object(pic(100))
    assert e.value.kind == "bad_answer"


# --- the other tasks (mocked network) -------------------------------------------------------------------------
def test_second_check_batches_and_filters_ids(tmp_path, home):
    net = FakeNet(anthropic_ok(dict(suspects=[dict(id="a", should_be="Box", confidence=0.8, reason="r"),
                                              dict(id="zzz", should_be="x", confidence=1, reason="made up")])))
    h = make(tmp_path, home, net)
    items = [dict(id=c, label="cup", crop=pic(50 + i)) for i, c in enumerate("abcde")]
    r = h.second_check(items, known_types=["cup", "box"], batch=3)
    assert len(net.calls) == 2 and r["checked"] == 5
    assert [s["id"] for s in r["suspects"]] == ["a"] and r["suspects"][0]["should_be"] == "box"


def test_draft_labels_low_trust_and_downscaled(tmp_path, home):
    net = FakeNet(anthropic_ok(dict(objects=[dict(x=0.5, y=0.25, confidence=0.99), dict(x=2, y=0.1), dict(x="bad", y=1)])))
    h = make(tmp_path, home, net)
    frame = np.full((1080, 1920, 3), 120, np.uint8)
    r = h.draft_labels(frame, "Cube", "printed cubes")
    assert r["trust"] == "low" and r["source"] == "ai" and len(r["labels"]) == 1
    lab = r["labels"][0]
    assert lab["cls"] == "cube" and lab["x"] == 960.0 and lab["y"] == 270.0 and lab["trust"] == "low" and lab["confidence"] <= 0.5
    sent = h.read_log()[0]["sent"]["images"][0]
    assert max(sent[0], sent[1]) <= ai.MAX_FRAME_SIDE


def test_setup_from_sentence_cleans_types(tmp_path, home):
    net = FakeNet(anthropic_ok(dict(classes=[dict(name="Printed Cube!", has_front=False, height_cm=3, note="top centre"),
                                             dict(name="cylinder", has_front=1), dict(name="unknown"), dict(name="cylinder")])))
    h = make(tmp_path, home, net)
    r = h.setup_from_sentence("we track printed cubes and cylinders on a table")
    assert [c["name"] for c in r["classes"]] == ["printed cube", "cylinder"]
    c0, c1 = r["classes"]
    assert c0["height_cm"] == 3.0 and c0["height_varies"] is False and c1["height_varies"] is True and c1["has_front"] is True
    with pytest.raises(ai.AIError):
        h.setup_from_sentence("   ")


# --- limits and cost -------------------------------------------------------------------------------------------
def test_crops_are_capped(tmp_path, home):
    net = FakeNet(anthropic_ok(dict(name="x", confidence=1, reason="")))
    h = make(tmp_path, home, net)
    big = np.random.RandomState(0).randint(0, 255, (1500, 1500, 3), np.uint8)
    h.name_object(big)
    w, hh, nbytes = h.read_log()[0]["sent"]["images"][0]
    assert max(w, hh) <= ai.MAX_CROP_SIDE and nbytes <= ai.MAX_IMAGE_BYTES


def test_too_many_pictures_refused_before_sending(tmp_path, home):
    net = FakeNet(anthropic_ok({}))
    h = make(tmp_path, home, net, max_images=3)
    with pytest.raises(ai.AIError) as e:
        h.object_or_background(pic(1), pic(2), strip=[pic(3), pic(4)])
    assert e.value.kind == "too_big" and net.calls == []


def test_estimate_cost_sends_nothing(tmp_path, home):
    net = FakeNet(anthropic_ok({}))
    h = make(tmp_path, home, net, enabled=False)
    est = h.estimate_cost("object_or_background", now=pic(10, 300), usual=pic(20, 300), strip=[pic(30)])
    assert est["n_images"] == 3 and est["tokens_in"] > 300 and est["cost_usd_max"] > 0
    assert net.calls == []
    h.configure(model="mystery-model")
    assert h.estimate_cost("name_object", crop=pic(5))["cost_usd_max"] is None
    h.configure(price_in=1.0, price_out=2.0)
    assert h.estimate_cost("name_object", crop=pic(5))["cost_usd_max"] > 0


# --- stand-in ----------------------------------------------------------------------------------------------------
def test_stand_in_needs_no_key_and_is_deterministic(tmp_path, home):
    h = make(tmp_path, home, provider="stand_in", key="")
    assert h.status()["sends_data"] is False and "nothing leaves" in h.status()["indicator"]
    a = h.object_or_background(pic(200, noise=3), pic(60, noise=3))
    b = h.object_or_background(pic(200, noise=3), pic(60, noise=3))
    assert a["verdict"] == b["verdict"] == "object"
    assert h.object_or_background(pic(60, noise=3), pic(62, noise=3))["verdict"] == "background"
    assert h.read_log()[0]["sent_to_service"] is False
    items = [dict(id="i%d" % i, label="cup", crop=pic(40)) for i in range(20)]
    r1, r2 = h.second_check(items), h.second_check(items)
    strip = lambda r: [{k: v for k, v in x.items() if k != "call_id"} for x in r["suspects"]]   # noqa: E731
    assert strip(r1) == strip(r2) and r1["checked"] == 20
    assert all(x["call_id"] in r1["calls"] for x in r1["suspects"])          # each suspect knows its call
    s = h.setup_from_sentence("we track printed cubes and cylinders on a table")
    assert [c["name"] for c in s["classes"]] == ["cube", "cylinder"]
    d = h.draft_labels(np.zeros((480, 640, 3), np.uint8), "cube")
    assert len(d["labels"]) == 2 and all(l["trust"] == "low" for l in d["labels"])
    assert h.name_object(pic(10))["source"] == "ai"


def test_off_switch_stops_stand_in_too(tmp_path, home):
    h = make(tmp_path, home, provider="stand_in", key="", enabled=False)
    with pytest.raises(ai.AIError):
        h.name_object(pic(10))
    assert h.read_log() == []


def test_plain_wording():
    src = open(ai.__file__).read().lower()
    assert "in real life" not in src
