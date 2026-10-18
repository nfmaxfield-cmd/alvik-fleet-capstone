#!/usr/bin/env python3
"""Fleet planner page: draw the floor, boards, keep-outs and obstacles,
give each robot its boards, press Plan, watch the plan play out, save.

    python tools/fleet/app.py --map map.json
    python tools/fleet/app.py --map map.json --plan plan.json --port 8770

The page is served only on this computer (127.0.0.1) and needs no internet.
`--map` is created (empty 122 x 122 cm floor) if it does not exist yet.
Save writes map.json and plan.json (next to the map unless --plan says
otherwise). Stop with Ctrl+C.
"""
import argparse
import json
import os
import sys
import threading
import traceback
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import fleetmap as FM   # noqa: E402
import planner as P     # noqa: E402


class Session:
    def __init__(self, map_path, plan_path):
        self.map_path = os.path.abspath(map_path)
        self.plan_path = os.path.abspath(plan_path)
        self.lock = threading.Lock()
        if os.path.exists(self.map_path):
            self.map = FM.load(self.map_path)
        else:
            self.map = FM.normalize({})
        self.plan = None
        if os.path.exists(self.plan_path):
            try:
                with open(self.plan_path, encoding="utf-8") as f:
                    self.plan = json.load(f)
            except (OSError, ValueError):
                self.plan = None

    def check(self, m):
        m = FM.normalize(m)
        body = FM.body_of(m)
        docks = {k: {"dock": FM.pose_out(v["dock"]), "predock": FM.pose_out(v["predock"]), "side": v["side"]}
                 for k, v in FM.docks(m).items()}
        return {"map": m, "docks": docks, "warnings": FM.validate(m), "hash": FM.map_hash(m),
                "robot": {"r_turn": body.r_turn, "r_side": body.r_side}}

    def state(self):
        out = self.check(self.map)
        out.update(plan=self.plan, map_path=self.map_path, plan_path=self.plan_path,
                   settings=dict(P.DEFAULT_SETTINGS),
                   plan_matches=bool(self.plan and self.plan.get("map_hash") == out["hash"]))
        return out


def make_handler(session):
    page_path = os.path.join(HERE, "page.html")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, body, ctype="application/json"):
            if isinstance(body, (dict, list)):
                body = json.dumps(body).encode()
            elif isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _body(self):
            n = int(self.headers.get("Content-Length") or 0)
            return json.loads(self.rfile.read(n).decode() or "{}") if n else {}

        def _guard(self, fn):
            try:
                fn()
            except (FM.MapError, P.PlanError, ValueError, KeyError, TypeError) as e:
                self._send(400, {"error": str(e)})
            except Exception as e:      # keep the server alive, show the problem in the page
                traceback.print_exc()
                self._send(500, {"error": "%s: %s" % (type(e).__name__, e)})

        def do_GET(self):
            self._guard(self._get)

        def do_POST(self):
            # Only this page may change files: another web page cannot send
            # JSON to 127.0.0.1 without the browser asking first (never
            # allowed here), and the Host check stops DNS tricks.
            host = (self.headers.get("Host") or "").split(":")[0]
            ctype = self.headers.get("Content-Type") or ""
            if host not in ("127.0.0.1", "localhost") or not ctype.startswith("application/json"):
                self._send(403, {"error": "refused: changes must come from the planner page"})
                return
            self._guard(self._post)

        def _get(self):
            u = urlparse(self.path)
            if u.path in ("/", "/index.html"):
                with open(page_path, "rb") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
            elif u.path == "/api/state":
                with session.lock:
                    self._send(200, session.state())
            else:
                self._send(404, {"error": "not found"})

        def _post(self):
            u = urlparse(self.path)
            b = self._body()
            s = session
            if u.path == "/api/check":
                self._send(200, s.check(b["map"]))
            elif u.path == "/api/plan":
                plan = P.make_plan(b["map"], over=b.get("settings") or {})
                self._send(200, {"plan": plan})
            elif u.path == "/api/event":
                # an obstacle appears at plan time t: who stops, then a new plan
                plan = b["plan"]
                t = float(b["t"])
                states = P.states_at(plan, t)
                m = b["map"]
                ob = b.get("obstacle")          # None: re-plan the rest, nothing new on the floor
                obs = []
                if ob:
                    ob.setdefault("source", "user")
                    ob.setdefault("time", FM.now_iso())
                    obs = [ob]
                over = dict(plan.get("settings") or {})
                if b.get("budget_s"):
                    over["replan_budget_s"] = float(b["budget_s"])
                newplan, events = P.obstacle_event(m, plan, states, obs, over=over)
                m2 = FM.normalize(dict(m, obstacles=list(m.get("obstacles") or []) + obs))
                self._send(200, {"plan": newplan, "events": events, "map": m2,
                                 "states": {k: dict(v, pose=FM.pose_out(v["pose"])) for k, v in states.items()}})
            elif u.path == "/api/save":
                with s.lock:
                    m = FM.save(b["map"], s.map_path)
                    s.map = m
                    saved = [s.map_path]
                    if b.get("plan"):
                        P.save_plan(b["plan"], s.plan_path)
                        s.plan = b["plan"]
                        saved.append(s.plan_path)
                self._send(200, {"saved": saved})
            else:
                self._send(404, {"error": "not found"})

    return Handler


def serve(session, port=8770):
    try:
        server = ThreadingHTTPServer(("127.0.0.1", port), make_handler(session))
    except OSError:
        raise SystemExit("Port %d is busy (is the page already open in another window?). "
                         "Close that, or start with --port %d" % (port, port + 1))
    server.daemon_threads = True
    return server, "http://127.0.0.1:%d/" % server.server_address[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", default="map.json", help="map file (created if missing)")
    ap.add_argument("--plan", default=None, help="plan file (default: plan.json next to the map)")
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()
    plan_path = args.plan or os.path.join(os.path.dirname(os.path.abspath(args.map)), "plan.json")
    session = Session(args.map, plan_path)
    server, url = serve(session, args.port)
    print("Map:  %s\nPlan: %s\nOpen %s  (Ctrl+C stops)" % (session.map_path, session.plan_path, url))
    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    sys.exit(main())
