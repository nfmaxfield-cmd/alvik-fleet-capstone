"""Every message the labelling page shows (toast(...) in page.html) must say how to take back what it
announces, or say plainly that nothing changed (so there is nothing to take back). A message for something
that truly can't be undone must be in CANT_UNDO below with the reason, and must say so itself.

The check is on the message's source text: a take-back phrase written in it (Z, Shift+Z, Esc, a button
that puts it back, "again" for the undo/redo words, "Nothing was changed" ...). Messages built only from a
variable or a server's error are listed in DYNAMIC with where their words come from."""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(os.path.dirname(HERE), "page.html")

# words that tell the person how to take it back, or that there is nothing to take back
TAKE_BACK = re.compile(
    r"\bZ\b|Shift\+Z|takes? (it|them|that|these|those|the \w+)? ?back|puts? (it|them|the [\w ]+)? ?back|brings? (it|them) back|"
    r"\bEsc\b|again\b|nothing (was |is )?(changed|added|sent|saved|accepted|investigated)|nothing changes|"
    r"no (label|frame) changed|labels? (are|is) (unchanged|kept|untouched)|nothing else changed|"
    r"only how it is shown|changes nothing|change nothing|nothing in the project is changed|"
    r"can't be (undone|taken back|brought back)|this can't be undone|"
    r"untick|tick the box|switches it (off|on)|turns it (off|on)|type \" ?\+ ?old|"
    r"unchanged|not saved yet|no change to save|changes it back|switches back|(was|were) not (changed|saved|made)|"
    r"nothing to change|"
    r"takes you back|Q (brings|to go back)|press Q|goes back|C (again|plays)|M again|B again|G and",
    re.I)

# messages that truly can't be undone: (a piece of the message's source, why) -- each says so itself
CANT_UNDO = [
    ("deleted from the project (this can't be taken back", "deleting a failed model: the page asks first and says it can't be undone"),
    ("is forgotten on this computer", "forgetting the AI key: the key is not kept anywhere, so there is nothing to put back (says so)"),
    ("The old sample is replaced and can't be brought back", "a new spot-check sample replaces the old one (asked first, says so)"),
]

# a message shown while something is under way; the message when it is done says how to take it back
INTERIM = ["Saving, and updating the guesses after this frame"]

# messages whose words come from a variable or from the app's answer
DYNAMIC = {
    "e.message, true": "an error from the app: every error the app sends says nothing was changed",
    "msg": "built just before the call, with its take-back part (see the lines above each)",
    "job.message + \" Your labels are unchanged; Learning curve shows the new model, and Guesses from switches to it.\", job.status === \"failed\"": "training ended (nothing to take back)",
}


def _calls(src, name="toast"):
    """(line, argument source) of every name(...) call, with nested parentheses and strings handled."""
    out = []
    for m in re.finditer(r"(?<![\w.])%s\(" % name, src):
        i = m.end()
        depth, j, q = 1, i, None
        while j < len(src) and depth:
            c = src[j]
            if q:
                if c == "\\":
                    j += 2
                    continue
                if c == q:
                    q = None
            elif c in "\"'`":
                q = c
            elif c == "(":
                depth += 1
            elif c == ")":
                depth -= 1
            j += 1
        out.append((src.count("\n", 0, m.start()) + 1, src[i:j - 1].strip()))
    return out


def _script():
    with open(PAGE, encoding="utf-8") as f:
        html = f.read()
    return html[html.index("<script>"):]


def test_every_message_says_how_to_take_it_back():
    src = _script()
    calls = [(ln, a) for ln, a in _calls(src) if not a.startswith("msg, err")]     # the toast function itself
    assert len(calls) > 150
    bad = []
    for ln, arg in calls:
        if arg in DYNAMIC:
            continue
        if any(piece in arg for piece, _ in CANT_UNDO):
            continue
        if TAKE_BACK.search(arg):
            continue
        if arg.endswith(", true") and re.search(r"\b(e\.message|j\.error|e\.data)\b", arg):
            continue            # an error from the app, passed on: the app's errors say what (nothing) changed
        if any(piece in arg for piece in INTERIM):
            continue
        bad.append("line %d: %s" % (ln, arg[:160]))
    assert not bad, "messages without a take-back part:\n" + "\n".join(bad)


def test_cant_undo_messages_say_so_themselves():
    src = _script()
    args = [a for _, a in _calls(src)]
    for piece, why in CANT_UNDO:
        hits = [a for a in args if piece in a]
        assert hits, "allow-listed message not found (remove it from CANT_UNDO): %s" % piece
        for a in hits:
            assert re.search(r"can't be (undone|taken back|brought back)|forgotten|can't be undone", a), (piece, why)


def test_listed_dynamic_messages_still_exist():
    args = {a for _, a in _calls(_script())}
    stale = [k for k in DYNAMIC if k not in args]
    assert not stale, "remove from DYNAMIC: %s" % stale
    for k in ("msg",):              # where the message is built first: its take-back words are in the build
        assert re.search(r"const msg = .*(Z takes|Z puts)", _script())


def test_the_scanner_catches_a_message_without_one():
    sample = 'x(); toast("Saved the thing on frame 3."); toast("Saved it. Z takes it back.");'
    found = [a for _, a in _calls(sample)]
    assert found == ['"Saved the thing on frame 3."', '"Saved it. Z takes it back."']
    assert not TAKE_BACK.search(found[0]) and TAKE_BACK.search(found[1])
