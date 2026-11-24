#!/usr/bin/env python3
"""(e24, the 24 Nov usability audit, finding 7) Update this Pi to GitHub's newest code with ONE command.

    ./run-fleet-pi.sh update            # the whole update
    ./run-fleet-pi.sh update check      # only say what it would do (nothing is changed)

What it does, in this order, saying each step in plain words:
  1. refuses while a run is going (nothing is changed);
  2. closes Mission Control (if it is open) and waits until its port is free;
  3. gets GitHub's newest code (git fetch); no network: stops here, nothing changed, Mission Control is started again;
  4. BACKS UP everything that is only on this Pi, before touching anything:
       - a branch  pi-backup-<date-time>  at the Pi's current commit (every commit made on this Pi stays on it),
       - a stash named the same for changed files that were not committed,
       - a folder  ~/mirlt-backups/<date-time>/  with a copy of every changed file, a patch of the changes, the list
         of the Pi's own commits, and any loose file that the new code would have overwritten;
  5. carries choices that belong to this Pi (camera 2 on or off, a capture rate set by hand) from an edited site file
     into the Pi's own file (<site folder>/site_local.json), unless GitHub's file now says the same;
  6. puts the code on GitHub's newest (origin/main) exactly. Loose files that are not part of the code (*.bak ...) stay;
  7. checks the result: the site file loads, the programs compile, the Safety group reads; if a check fails it puts
     the Pi back exactly as it was (the backup branch and the stash) and says FAIL;
  8. starts Mission Control again if it was open, and waits until it answers;
  9. prints PASS or FAIL with what to do next, and where every backup is.

It never deletes a local change: a commit, a changed file or a loose file is always in the branch, the stash or the
folder, and the last lines say where. To look at a backup later:   git log pi-backup-<date-time>
To put the Pi back as it was before the update:                    git reset --hard pi-backup-<date-time>

Changelog
  2026-11-24  first version (e24). Rehearsed on a scratch clone rebuilt like the school Pi on 23 Nov (four local
              commits on top of 294d937, an edited site file, loose .bak files): docs/build-reports/e24-...md.
"""
import datetime as dt
import json
import os
import shutil
import subprocess
import sys
import time

VERSION = "update-pi-2026-11-24"
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
if not os.path.isdir(os.path.join(REPO, ".git")):
    # the FIRST update of a Pi that does not have this file yet runs a copy fetched to another place
    # (git show origin/main:tools/pi/update_pi.py > ~/update_pi.py; python3 ~/update_pi.py): the folder it is typed in
    REPO = os.getcwd()
REPO = os.path.abspath(os.environ.get("MIRLT_UPDATE_REPO") or REPO)
MC_PORT = int(os.environ.get("MIRLT_MC_PORT") or 8095)
PORT_WAIT_S = 40.0      # [chosen] how long Mission Control gets to close or to answer (it closes in about 5 s on the Pi
#                         and opens in about 10). DOE: no (a patience; a longer wait only delays the FAIL words)
RUN_SESSIONS = ("visit", "calib", "freeroam", "doe", "monday", "practice")
LINES = []


def say(text=""):
    print(text, flush=True)
    LINES.append(text)


def git(*args, check=True, cwd=None):
    r = subprocess.run(["git"] + list(args), cwd=cwd or REPO, capture_output=True, text=True)
    if check and r.returncode != 0:
        raise RuntimeError("git %s: %s" % (" ".join(args), (r.stderr or r.stdout).strip()[:400]))
    return r


def port_open(port=None):
    import socket
    s = socket.socket()
    s.settimeout(1.0)
    try:
        return s.connect_ex(("127.0.0.1", port or MC_PORT)) == 0
    finally:
        s.close()


def tmux_has(name):
    try:
        return subprocess.run(["tmux", "has-session", "-t", name], capture_output=True).returncode == 0
    except OSError:
        return False


def run_going():
    """Words when a run is going (nothing may be updated then), else ''."""
    if os.environ.get("MIRLT_UPDATE_NO_MC") == "1":
        return ""
    if port_open():
        try:
            import urllib.request
            with urllib.request.urlopen("http://127.0.0.1:%d/api/start" % MC_PORT, timeout=4) as r:
                st = (json.load(r).get("status") or {})
            if st.get("running") or st.get("stopping"):
                return "Mission Control is running %s" % (st.get("title") or "a run")
        except Exception:  # noqa: BLE001  (an old Mission Control that does not answer this: the tmux check below)
            pass
    for s in RUN_SESSIONS:
        if tmux_has(s):
            r = subprocess.run(["tmux", "list-panes", "-t", s, "-F", "#{pane_dead} #{pane_current_command}"],
                               capture_output=True, text=True)
            if "python" in r.stdout:
                return "a run is going in the terminal (tmux session %s)" % s
    return ""


def close_mc():
    """Close Mission Control and wait until its port is free. Returns (was open, closed)."""
    if os.environ.get("MIRLT_UPDATE_NO_MC") == "1":
        return False, True
    was = port_open()
    if not was and not tmux_has("mc"):
        return False, True
    if tmux_has("mc"):
        subprocess.run(["tmux", "send-keys", "-t", "mc", "C-c"], capture_output=True)
    t0 = time.time()
    while time.time() - t0 < PORT_WAIT_S:
        if not port_open():
            subprocess.run(["tmux", "send-keys", "-t", "mc", "Enter"], capture_output=True)
            time.sleep(0.5)
            subprocess.run(["tmux", "kill-session", "-t", "mc"], capture_output=True)
            return was, True
        time.sleep(1.0)
    return was, False


def open_mc():
    if os.environ.get("MIRLT_UPDATE_NO_MC") == "1":
        return True
    subprocess.run(["bash", os.path.join(REPO, "run-fleet-pi.sh"), "mc"], cwd=REPO, capture_output=True, text=True)
    t0 = time.time()
    while time.time() - t0 < PORT_WAIT_S:
        if port_open():
            return True
        time.sleep(1.0)
    return False


def site_files_edited(base):
    """Tracked site files whose text on this Pi differs from the commit the Pi's own work started from (`base`):
    [(path, the Pi's text, the base's text)]."""
    out = []
    r = git("ls-files", "tools/fleet/sites", check=False)
    for p in r.stdout.split():
        if not p.endswith(".json"):
            continue
        try:
            with open(os.path.join(REPO, p)) as f:
                mine = f.read()
        except OSError:
            continue
        b = git("show", "%s:%s" % (base, p), check=False)
        if b.returncode == 0 and b.stdout != mine:
            out.append((p, mine, b.stdout))
    return out


def checks():
    """The result checked: [(what, ok, words)]."""
    res = []
    env = dict(os.environ)
    r = subprocess.run([sys.executable, os.path.join(REPO, "tools", "fleet", "sites.py")], cwd=REPO, capture_output=True,
                       text=True, env=env)
    res.append(("the site file loads", r.returncode == 0, (r.stdout.strip().splitlines() or [r.stderr.strip()[:300]])[0]))
    files = [os.path.join(REPO, "tools", "fleet", f) for f in ("mission_control.py", "mission_run.py", "run_plan.py",
                                                               "start_job.py", "tag_bridge.py", "sites.py")]
    r = subprocess.run([sys.executable, "-m", "py_compile"] + [f for f in files if os.path.exists(f)], cwd=REPO,
                       capture_output=True, text=True)
    res.append(("the programs compile", r.returncode == 0, (r.stderr.strip().splitlines() or ["ok"])[-1][:200]))
    r = subprocess.run([sys.executable, os.path.join(REPO, "tools", "fleet", "safety_group.py"), "show"], cwd=REPO,
                       capture_output=True, text=True, env=env)
    res.append(("the Safety group reads", r.returncode == 0, (r.stdout.strip().splitlines() or [r.stderr[:200]])[-1][:200]))
    r = subprocess.run(["bash", "-n", os.path.join(REPO, "run-fleet-pi.sh")], capture_output=True, text=True)
    res.append(("the launcher script is whole", r.returncode == 0, (r.stderr.strip() or "ok")[:200]))
    return res


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    only_check = bool(argv and argv[0] in ("check", "--check", "dry", "--dry-run"))
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    backup_dir = os.path.join(os.path.expanduser(os.environ.get("MIRLT_BACKUP_DIR") or "~/mirlt-backups"), stamp)
    branch = "pi-backup-" + stamp
    n_ = 1
    while git("rev-parse", "--verify", "-q", "refs/heads/" + branch, check=False).returncode == 0:
        n_ += 1                        # two updates in the same second: never an existing backup's name
        branch = "pi-backup-%s-%d" % (stamp, n_)
    say("UPDATE: bringing this Pi to GitHub's newest code%s." % (" (check only: nothing is changed)" if only_check else ""))
    kept = []          # where local things were kept, for the last lines
    local_before = {}  # each site_local.json this update wrote: what it held before (None: it did not exist)

    def local_back():
        for lp_, was in local_before.items():
            try:
                if was is None:
                    os.remove(lp_)
                else:
                    with open(lp_, "w") as f_:
                        json.dump(was, f_, indent=1)
            except OSError:
                pass

    def fail(words, nxt):
        say("")
        say("FAIL: %s" % words)
        say("What to do: %s" % nxt)
        for k in kept:
            say("  kept: %s" % k)
        return 1

    # 1. a run going?
    rg = run_going()
    if rg:
        return fail("%s. Nothing was changed." % rg, "stop the run (the red STOP button, or ./run-fleet-pi.sh stopall), "
                    "then type ./run-fleet-pi.sh update again.")
    try:
        cur = git("rev-parse", "--abbrev-ref", "HEAD").stdout.strip()
        head = git("rev-parse", "HEAD").stdout.strip()
    except (RuntimeError, OSError) as e:
        return fail("this folder is not the code's git folder (%s). Nothing was changed." % e,
                    "type cd ~/alvik-fleet-capstone and then ./run-fleet-pi.sh update")
    # 2. Mission Control closed
    was_open = False
    if not only_check:
        say("1. Closing Mission Control (it is started again at the end) ...")
        was_open, closed = close_mc()
        if not closed:
            return fail("Mission Control did not close within %.0f s. Nothing was changed." % PORT_WAIT_S,
                        "type tmux attach -t mc, press Ctrl+C there, then ./run-fleet-pi.sh update again.")
        say("   %s" % ("closed." if was_open else "it was not open."))
    # 3. fetch
    say("2. Getting GitHub's newest code ...")
    r = git("fetch", "origin", check=False)
    if r.returncode != 0:
        if was_open:
            open_mc()
        return fail("GitHub could not be reached (%s). Nothing was changed%s." % (
            (r.stderr.strip().splitlines() or ["no answer"])[-1][:160], "; Mission Control is open again" if was_open else ""),
                    "check the Pi's internet (ping github.com), then type ./run-fleet-pi.sh update again.")
    target = git("rev-parse", "origin/main").stdout.strip()
    tsub = git("log", "-1", "--format=%h %s", "origin/main").stdout.strip()
    base = git("merge-base", "HEAD", "origin/main", check=False).stdout.strip() or head
    own = [ln for ln in git("log", "--format=%h %s (%an)", "%s..HEAD" % target, "--no-merges").stdout.splitlines() if ln]
    changed = [ln for ln in git("status", "--porcelain").stdout.splitlines() if ln and not ln.startswith("??")]
    loose = [ln[3:] for ln in git("status", "--porcelain").stdout.splitlines() if ln.startswith("??")]
    incoming = set(git("ls-tree", "-r", "--name-only", "origin/main").stdout.split("\n"))
    clash = [p for p in loose if p in incoming or any(q.startswith(p) for q in incoming if p.endswith("/"))]
    say("   GitHub's newest: %s" % tsub)
    say("   This Pi: %s%s%s" % (git("log", "-1", "--format=%h %s").stdout.strip(),
                                 "; %d commit%s made on this Pi" % (len(own), "" if len(own) == 1 else "s") if own else "",
                                 "; %d changed file%s not committed" % (len(changed), "" if len(changed) == 1 else "s")
                                 if changed else ""))
    edits = site_files_edited(base)
    if head == target and not changed:
        say("   Already on GitHub's newest code: nothing to update.")
        if not only_check:
            if was_open:
                say("3. Starting Mission Control again ...")
                open_mc()
            say("")
            say("PASS: this Pi already has GitHub's newest code (%s). Nothing was changed." % tsub)
            return 0
    if only_check:
        for ln in own:
            say("   would keep (branch %s): %s" % (branch, ln))
        for ln in changed:
            say("   would keep (stash + %s): %s" % (backup_dir, ln[3:]))
        for p in clash:
            say("   would move out of the new code's way (into %s): %s" % (backup_dir, p))
        for p, _m, _b in edits:
            say("   would carry this Pi's own choices from %s into the Pi's own file" % p)
        say("")
        say("CHECK ONLY: nothing was changed. To update: ./run-fleet-pi.sh update")
        return 0
    # 4. backups, before anything is touched
    say("3. Backing up everything that is only on this Pi ...")
    try:
        git("branch", branch, head)
        kept.append("the Pi's commits: branch %s (look: git log %s)" % (branch, branch))
        os.makedirs(backup_dir, exist_ok=True)
        with open(os.path.join(backup_dir, "about.txt"), "w") as f:
            f.write("Backup made by ./run-fleet-pi.sh update on %s\nThe Pi was at %s on branch %s\nGitHub's newest was %s\n"
                    "The Pi's own commits (not on GitHub):\n%s\n\nTo put the Pi back as it was: git reset --hard %s ; "
                    "git stash list (the stash named %s holds the uncommitted changes: git stash pop)\n" % (
                        stamp, head, cur, target, "\n".join(own) or "(none)", branch, branch))
        if own:
            with open(os.path.join(backup_dir, "pi-commits.patch"), "w") as f:
                f.write(git("format-patch", "--stdout", "%s..%s" % (base, head), check=False).stdout)
        if changed:
            with open(os.path.join(backup_dir, "uncommitted.patch"), "w") as f:
                f.write(git("diff", "HEAD").stdout)
            for ln in changed:
                p = ln[3:].split(" -> ")[-1].strip().strip('"')
                src = os.path.join(REPO, p)
                if os.path.isfile(src):
                    dst = os.path.join(backup_dir, "changed-files", p)
                    os.makedirs(os.path.dirname(dst), exist_ok=True)
                    shutil.copy2(src, dst)
        kept.append("copies and patches: %s" % backup_dir)
    except (RuntimeError, OSError) as e:
        if was_open:
            open_mc()
        return fail("the backup could not be made (%s). Nothing else was changed." % e,
                    "check the card's free space (./run-fleet-pi.sh space), then try again.")
    say("   branch %s; folder %s" % (branch, backup_dir))
    # 6. onto origin/main
    say("4. Putting the code on GitHub's newest ...")
    stashed = False
    try:
        if changed:
            r = git("stash", "push", "-m", branch, check=False)
            stashed = r.returncode == 0 and "No local changes" not in r.stdout
            if stashed:
                kept.append("the uncommitted changes: git stash (named %s; see: git stash list)" % branch)
        for p in clash:
            src = os.path.join(REPO, p)
            dst = os.path.join(backup_dir, "loose-files", p)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.move(src, dst)
            say("   a loose file in the new code's way was moved to the backup: %s" % p)
        if cur != "main":
            git("checkout", "-B", "main", "origin/main")
        else:
            git("reset", "--hard", "origin/main")
    except (RuntimeError, OSError) as e:
        git("reset", "--hard", branch, check=False)
        if stashed:
            git("stash", "pop", check=False)
        if was_open:
            open_mc()
        return fail("the code could not be switched (%s). The Pi is back as it was." % e,
                    "send Nick the lines above.")
    say("   now at: %s" % git("log", "-1", "--format=%h %s").stdout.strip())
    # 5. this Pi's own choices out of an edited site file (read with the NEW code's sites.py, after the switch)
    if edits:
        say("5. Carrying this Pi's own choices out of the edited site file (into the Pi's own file) ...")
        sys.path.insert(0, os.path.join(REPO, "tools", "fleet"))
        for p, mine, base_text in edits:
            name = os.path.basename(p)[:-5]
            new_text = git("show", "origin/main:%s" % p, check=False).stdout
            try:
                import sites as SI
                got, other = SI.local_from_edits(name, mine, base_text)
                # what GitHub's file now says anyway needs no copy on the Pi
                same, _ = SI.local_from_edits(name, mine, new_text) if new_text else (got, [])
                got = {k: v for k, v in got.items() if k in same}
                conf = os.path.expanduser(json.loads(mine).get("conf_dir") or (
                    "~/.config/mirlt" if name == "home" else "~/.config/mirlt/sites/" + name))
                lp = os.path.join(conf, SI.LOCAL_FILE)
                cur_local = {}
                try:
                    with open(lp) as f:
                        cur_local = json.load(f)
                except (OSError, ValueError):
                    pass
                local_before.setdefault(lp, dict(cur_local) if os.path.exists(lp) else None)
                for k, v in sorted(got.items()):
                    cur_local[k] = v
                    say("   kept on this Pi: %s = %s (%s)" % (k, json.dumps(v), lp))
                if got:
                    os.makedirs(conf, exist_ok=True)
                    with open(lp, "w") as f:
                        json.dump(cur_local, f, indent=1)
                    kept.append("this Pi's own choices: %s" % lp)
                upstream = [k for k in SI.local_from_edits(name, mine, base_text)[0] if k not in got]
                for k in upstream:
                    say("   %s: GitHub's file now says the same as this Pi did: nothing to keep" % k)
                # a value this Pi ran with that GitHub's file now sets differently, and that was not an edit of this
                # Pi's (it equals what the Pi started from): GitHub's is used, and the line to keep the old one is said
                for k, v in sorted(same.items()):
                    if k not in got:
                        say("   %s: this Pi ran with %s (the value it was given, not an edit made here); GitHub's file "
                            "now says otherwise and GitHub's is used. To keep %s on this Pi: ./run-fleet-pi.sh local "
                            "set %s %s" % (k, json.dumps(v), json.dumps(v), k, json.dumps(v)))
                for w in other:
                    say("   NOT carried over (it is in the backup; tell Nick if it matters): %s" % w)
            except Exception as e:  # noqa: BLE001  (the backup has the file whatever happens here)
                say("   (%s: its edits could not be read as choices (%s); the file itself is in the backup)" % (p, e))
    else:
        say("5. No site file was edited on this Pi: nothing to carry.")
    # 7. checks
    say("6. Checking the result ...")
    bad = []
    for what, ok, words in checks():
        say("   %s: %s%s" % (what, "ok" if ok else "NOT OK", "" if ok else " (%s)" % words))
        if not ok:
            bad.append(what)
    if bad:
        git("reset", "--hard", branch, check=False)
        if stashed:
            git("stash", "pop", check=False)
        local_back()
        if was_open:
            open_mc()
        return fail("the new code did not pass its checks (%s). The Pi was put back exactly as it was before the "
                    "update%s." % (", ".join(bad), "; Mission Control is open again" if was_open else ""),
                    "send Nick the lines above; the Pi runs as before meanwhile.")
    # 8. Mission Control again
    if was_open:
        say("7. Starting Mission Control again ...")
        if not open_mc():
            say("")
            say("PASS (the code is updated), BUT Mission Control did not answer within %.0f s." % PORT_WAIT_S)
            say("What to do: type ./run-fleet-pi.sh mc")
            for k in kept:
                say("  kept: %s" % k)
            return 2
        say("   open.")
    say("")
    say("PASS: this Pi is on GitHub's newest code (%s)." % tsub)
    if own:
        say("%d commit%s made on this Pi %s not on GitHub; %s kept on the branch %s:" % (
            len(own), "" if len(own) == 1 else "s", "is" if len(own) == 1 else "are", "it is" if len(own) == 1 else
            "they are", branch))
        for ln in own:
            say("  %s" % ln)
    for k in kept:
        say("  kept: %s" % k)
    if loose:
        say("  left where they were (not part of the code): %s" % ", ".join(p for p in loose if p not in clash))
    say("Nothing was deleted. To put the Pi back as it was before this update: git reset --hard %s" % branch)
    try:
        with open(os.path.join(backup_dir, "update-log.txt"), "w") as f:
            f.write("\n".join(LINES) + "\n")
    except OSError:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
