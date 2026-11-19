#!/usr/bin/env python3
"""(e23, 19 Nov, offline only) The Stop-before-the-robots-are-named case of the school runs (16 Nov 17:43, 18 Nov
16:53), played through the stop watcher of the code in REPO with the watcher's own clock stepped by hand: the site's
slots are Alpha / Bravo with no address, the robots answer as AL-E7DC / AL-EB54. Prints how many `stop` messages the
watcher sent in the 5 minutes after one press, and whether it ever finished.
    stop_case.py REPO [MINUTES]
Version 1 (2026-11-19). No robot, no port: listen and send_stop are stand-ins."""
import json, os, sys
REPO = os.path.abspath(sys.argv[1])
MIN = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
sys.path.insert(0, os.path.join(REPO, "tools", "fleet"))
import stop_confirm as SC   # noqa: E402
sent, t = [], [0.0]
guess = [{"name": "Alpha", "ref": "", "ip": None, "bound": False}, {"name": "Bravo", "ref": "", "ip": None, "bound": False}]
sw = SC.StopWatch([SC.Job("visit", None, None, guess)], practice=False,
                  listen=lambda s: {"AL-E7DC": "192.168.1.112", "AL-EB54": "192.168.1.126"},
                  send_stop=lambda ips: sent.append(round(t[0], 1)), retry_s=3.0, clock=lambda: t[0])
done_at = None
k = 0
while t[0] <= MIN * 60.0:
    r = sw.step()
    if r is True and done_at is None:
        done_at = t[0]
    k += 1
    t[0] = 0.5 * k
st = sw.state()
print(json.dumps({"code": os.path.basename(REPO), "minutes": MIN, "stop_messages_sent": len(sent),
                  "last_stop_sent_at_s": sent[-1] if sent else None, "watch_finished_at_s": done_at,
                  "phase": st.get("phase"), "robots": [[r.get("name"), r.get("state")] for r in st.get("robots", [])]}))
