"""Build e24 (24 Nov 2026): `./run-fleet-pi.sh update` (tools/pi/update_pi.py) rehearsed on a scratch clone rebuilt
like the school Pi on 23 Nov (docs/results/e24-school-usability/update_rehearsal.sh): Jay-Sun's three local commits
and a merge on top of 294d937, an edited site file that was not committed, two loose .bak files. Offline: a bare copy
of this repo stands in for GitHub. Skipped where the repo's history does not hold those commits (a shallow clone)."""
import os
import subprocess

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
SCRIPT = os.path.join(REPO, "docs", "results", "e24-school-usability", "update_rehearsal.sh")


def _has(commit):
    return subprocess.run(["git", "cat-file", "-e", commit + "^{commit}"], cwd=REPO, capture_output=True).returncode == 0


def _committed():
    """The rehearsal clones this repo's HEAD: the update command must be committed there."""
    r = subprocess.run(["git", "cat-file", "-e", "HEAD:tools/pi/update_pi.py"], cwd=REPO, capture_output=True)
    return r.returncode == 0


needs_history = pytest.mark.skipif(not (_has("294d937") and _has("b202ea5") and _committed()),
                                   reason="needs the repo's history back to b202ea5 and update_pi.py committed")


def _run(tmp_path, edit):
    r = subprocess.run(["bash", SCRIPT, str(tmp_path / "w"), edit], capture_output=True, text=True, timeout=300)
    return r.stdout + r.stderr


@needs_history
def test_the_update_brings_a_diverged_pi_to_github_and_loses_nothing(tmp_path):
    out = _run(tmp_path, "other")
    assert "PASS: this Pi is on GitHub's newest code" in out
    for line in ("HEAD == GitHub's newest: yes", "holds the Pi's old HEAD: yes", "Jay-Sun's 3 commits on it: 3",
                 "byte-identical in the backup folder: yes", "loose .bak files still in place and unchanged: yes",
                 "working tree clean of tracked changes: yes"):
        assert line in out, line
    assert "stash kept: stash@{0}" in out
    assert "NOT carried over (it is in the backup; tell Nick if it matters): edge_in_cm_was_edited_here" in out
    assert "PASS: this Pi already has GitHub's newest code" in out           # the second update changes nothing


@needs_history
def test_a_choice_made_on_the_pi_is_kept_on_the_pi(tmp_path):
    """Camera 2 switched on in a commit on the Pi and off again by hand: off is this Pi's choice, and stays."""
    out = _run(tmp_path, "sideoff")
    assert "kept on this Pi: cameras.side.use = false" in out and "Camera 2 (side): off." in out
    assert "PASS: this Pi is on GitHub's newest code" in out


def test_the_launcher_answers_a_mistyped_command_in_a_few_lines():
    env = dict(os.environ, HOME=str(os.environ.get("TMPDIR", "/tmp")))
    r = subprocess.run(["bash", os.path.join(REPO, "run-fleet-pi.sh"), "mcstop"], capture_output=True, text=True, env=env)
    assert r.returncode == 1 and "Did you mean mc-stop?" in r.stdout and len(r.stdout.splitlines()) <= 16
    r = subprocess.run(["bash", os.path.join(REPO, "run-fleet-pi.sh"), "help"], capture_output=True, text=True, env=env)
    assert r.returncode == 0 and len(r.stdout.splitlines()) == 12 and "update" in r.stdout and "stopall" in r.stdout
    r = subprocess.run(["bash", os.path.join(REPO, "run-fleet-pi.sh"), "stop"], capture_output=True, text=True, env=env,
                       stdin=subprocess.DEVNULL)
    assert "It does NOT stop the robots" in r.stdout and "stopall" in r.stdout and "Nothing was stopped." in r.stdout
