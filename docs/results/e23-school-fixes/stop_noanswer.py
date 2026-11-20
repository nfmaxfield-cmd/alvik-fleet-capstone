"""(e23c, from the audit's stop_noanswer.py) A Stop before the robots are named that NO robot answers: what the
watcher sends and what the page says. usage: python3 stop_noanswer.py REPO   (stubs only: nothing is sent anywhere)"""
import os, sys, tempfile
os.environ["HOME"] = tempfile.mkdtemp()
sys.path.insert(0, os.path.join(sys.argv[1], "tools", "fleet"))
import stop_confirm as SC
t, sent = [0.0], []
guess = [{"name": "Alpha", "ref": "", "ip": None, "bound": False}, {"name": "Bravo", "ref": "", "ip": None, "bound": False}]
sw = SC.StopWatch([SC.Job("visit", None, None, guess)], practice=False, listen=lambda s: {},
                  send_stop=lambda ips: sent.append(t[0]), retry_s=3.0, clock=lambda: t[0])
print("version:", SC.VERSION)
for k in range(20):
    done = sw.step()
    if k in (0, 6, 12, 19):
        print("%4.1f s after Stop  `stop` sent %d time%s  page: %s" % (t[0], len(sent), "" if len(sent) == 1 else "s", sw.state()["words"]))
    t[0] += 0.5
for r in sw.state()["robots"]:
    print("  %s: [%s] %s" % (r["name"], r["state"], r["words"]))
