"""linkloss.py <folder with unzipped session logs> [out.json]

Command loss on the radio link in every real E02 session (2026-10-26 runner fixes, item 1e), and how it relates to
holds and failed moves. For each session folder (e02-YYYYMMDD-HHMMSS, with robot-logs-*/clock_sync.csv):
  - ping loss: the runner numbers its clock pings with one counter for all robots (cN), so a number no robot
    answered is a ping or its answer lost; overall and per 5-minute window;
  - round-trip times of the answered pings (ms): the robots' normal reply time;
  - move sends (go / turn with an id, e02_log.txt) with no record at all from the robot (leg.csv) before the next
    send of that id (sends.py of the 21:14 analysis);
  - holds (e02_state.json `holds`: kind lost / border / ...) and failed legs (leg.csv result failed), placed in
    the 5-minute windows.
Writes a JSON summary (default linkloss.json next to this file) used by figs.py and the build report."""
import csv, glob, json, os, re, sys, datetime as dt
from collections import defaultdict

ROOT = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(os.path.abspath(__file__)), 'linkloss.json')
WIN = 300.0


def sessions(root):
    best = {}
    for f in glob.glob(os.path.join(root, '**', 'robot-logs-*', 'clock_sync.csv'), recursive=True):
        d = os.path.dirname(os.path.dirname(f))
        name = os.path.basename(d)
        if not re.match(r'e02-\d{8}-\d{6}$', name):
            continue
        n = len(os.listdir(d))
        if name not in best or n > best[name][1]:
            best[name] = (d, n)
    return sorted((k, v[0]) for k, v in best.items())


def read(path):
    try:
        with open(path, newline='') as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


def log_sends(d, start):
    """[(t, robot, id, kind)] of go / turn sends in e02_log.txt (times to the second, local; day rollover)."""
    out, day, last = [], start.date(), None
    try:
        lines = open(os.path.join(d, 'e02_log.txt'), encoding='utf-8', errors='replace')
    except OSError:
        return out
    for line in lines:
        m = re.match(r'\[(\d\d):(\d\d):(\d\d)\]\s+-> (\w+): (go|turn) (\S+) (\d+)', line)
        if not m:
            continue
        hms = tuple(int(m.group(i)) for i in (1, 2, 3))
        if last is not None and hms < last and last[0] >= 20 and hms[0] <= 6:
            day = day + dt.timedelta(days=1)
        last = hms
        t = dt.datetime(day.year, day.month, day.day, *hms).astimezone().timestamp()
        out.append((t, m.group(4), m.group(7), m.group(5)))
    return out


def one(name, d):
    start = dt.datetime.strptime(name[4:], '%Y%m%d-%H%M%S')
    clock, legs = [], []
    for sub in sorted(glob.glob(os.path.join(d, 'robot-logs-*'))):
        clock += read(os.path.join(sub, 'clock_sync.csv'))
        legs += read(os.path.join(sub, 'leg.csv'))
    tok = {}
    rtt = []
    for r in clock:
        try:
            tok[int(r['token'].lstrip('c'))] = float(r['t_send'])
            rtt.append(float(r['rtt_ms']))
        except (ValueError, KeyError):
            pass
    if len(tok) < 20:
        return None
    ks = sorted(tok)
    # pings per window: numbers spanned between the first and last answered in the window
    win = defaultdict(lambda: dict(lo=10 ** 12, hi=-1, n=0))
    t0 = tok[ks[0]]
    for k in ks:
        w = win[int((tok[k] - t0) // WIN)]
        w['lo'], w['hi'], w['n'] = min(w['lo'], k), max(w['hi'], k), w['n'] + 1
    # move sends against leg records
    rec = defaultdict(list)
    for r in legs:
        try:
            rec[(r['robot'], r['id'])].append((float(r['t_recv']), r.get('result') or r.get('state') or ''))
        except (ValueError, KeyError):
            pass
    sends = log_sends(d, start)
    by = defaultdict(list)
    for t, rb, lid, kind in sends:
        by[(rb, lid)].append(t)
    lost_sends = []
    for k, ts in by.items():
        for i, t in enumerate(ts):
            nxt = ts[i + 1] if i + 1 < len(ts) else t + 60
            if not [x for x in rec.get(k, []) if t - 1.5 <= x[0] < nxt - 0.5]:
                lost_sends.append(t)
    # does the log's clock agree with the robots' t_recv? (second-resolution log; local time zone of this machine)
    failed = [float(r['t_recv']) for r in legs if (r.get('result') or '') == 'failed']
    try:
        st = json.load(open(os.path.join(d, 'e02_state.json')))
    except (OSError, ValueError):
        st = {}
    holds = [(h[0], h[1], h[2]) for h in (st.get('holds') or []) if isinstance(h, list) and len(h) >= 3]
    wins = []
    for b in sorted(win):
        w = win[b]
        span = w['hi'] - w['lo'] + 1
        a, z = t0 + WIN * b, t0 + WIN * (b + 1)
        if span < 15:
            continue
        wins.append(dict(t=a, pings=span, lost=span - w['n'], loss=(span - w['n']) / span,
                         sends=sum(1 for s in sends if a <= s[0] < z),
                         lost_sends=sum(1 for s in lost_sends if a <= s < z),
                         failed_legs=sum(1 for s in failed if a <= s < z),
                         holds=sum(1 for h in holds if a <= h[0] < z),
                         lost_holds=sum(1 for h in holds if a <= h[0] < z and h[2] == 'lost')))
    span = ks[-1] - ks[0] + 1
    rs = sorted(rtt)
    pct = lambda p: rs[min(len(rs) - 1, int(p * (len(rs) - 1)))]
    return dict(session=name, pings=span, lost=span - len(ks), loss=(span - len(ks)) / span,
                rtt_ms=dict(n=len(rs), p50=pct(.5), p90=pct(.9), p99=pct(.99), p999=pct(.999), max=rs[-1]),
                sends=len(sends), lost_sends=len(lost_sends), failed_legs=len(failed),
                holds=len(holds), lost_holds=sum(1 for h in holds if h[2] == 'lost'),
                hold_kinds=sorted({h[2] for h in holds}), windows=wins)


res = [r for r in (one(n, d) for n, d in sessions(ROOT)) if r]
for r in res:
    q = r['rtt_ms']
    print('%s  pings %5d lost %5.1f%%  rtt p50 %5.1f p99 %6.1f p99.9 %6.1f max %6.0f ms  sends %4d lost %3d  failed %3d '
          'holds %2d (lost %d)' % (r['session'], r['pings'], 100 * r['loss'], q['p50'], q['p99'], q['p999'], q['max'],
                                   r['sends'], r['lost_sends'], r['failed_legs'], r['holds'], r['lost_holds']))
json.dump(res, open(OUT, 'w'), indent=1)
print('->', OUT)
