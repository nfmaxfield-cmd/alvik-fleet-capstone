"""figs.py OUTDIR PREV : the report's three figures.
  e02-oct25-2114-corner.png  Left at the bottom-right corner, video crops with the stored calibration's lines
  e02-oct25-2114-link.png    commands lost on the way to the robots (clock pings no robot answered), per 15 minutes,
                             this session against the two afternoon sessions (PREV: folder with their unzipped logs)
  e02-oct25-2114-speed.png   the runner's pictures a second against the recorder's, per minute, phases shaded"""
import sys, os, csv, collections, statistics as S, bisect
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
import cv2, numpy as np
import common, pose
from common import ep, lt
out, prev = sys.argv[1], sys.argv[2]
INK, MUTED, GRID, BAND = '#3d3c38', '#6b6a64', '#e4e3df', '#efeeea'
C1, C2, C3 = '#2a78d6', '#eb6834', '#1baf7a'


def style(ax):
    for s in ax.spines.values():
        s.set_visible(False)
    ax.grid(axis='y', color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(labelsize=8, colors=MUTED, length=0)


# 1. the corner strip
d = common.W + '/f_left'
fr = sorted((float(t), int(k)) for k, t in (l.strip().split(',') for l in open(d + '/times.csv')))
ts = [f[0] for f in fr]
shots = [('21:42:44.5', 'ray 6 nears the right edge'), ('21:42:47.0', 'edge stop; back-out held'),
         ('21:44:58.0', 'after 2 holds, 104 s'), ('21:45:01.9', 'turn 88 deg swings it'),
         ('21:45:37.9', 'identify turn: slides on'), ('21:46:09.9', 'left out (hung, 10 deg tilt)')]
tiles = []
for s, lab in shots:
    i = min(range(len(ts)), key=lambda j: abs(ts[j] - ep(s)))
    im = cv2.imread('%s/%06d.jpg' % (d, fr[i][1]))
    for off, col in ((0, (40, 40, 220)), (-4, (214, 120, 42))):
        for pts in ([(off, y) for y in np.linspace(-10, 40, 20)], [(x, off) for x in np.linspace(-10, 40, 20)]):
            p = [tuple(int(round(c)) for c in pose.board_px(x, y)) for x, y in pts]
            for a, b in zip(p, p[1:]):
                cv2.line(im, a, b, col, 1, cv2.LINE_AA)
    c = cv2.resize(im[285:350, 355:430], None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC)
    tiles.append((c, lt(ts[i])[:10], lab))
fig, axs = plt.subplots(1, 6, figsize=(12, 2.9), dpi=130)
for ax, (c, t, lab) in zip(axs, tiles):
    ax.imshow(cv2.cvtColor(c, cv2.COLOR_BGR2RGB))
    ax.set_axis_off()
    ax.set_title('%s\n%s' % (t, lab), fontsize=8, color=INK)
fig.tight_layout()
fig.savefig(os.path.join(out, 'e02-oct25-2114-corner.png'))

# 2. link loss
def loss(dd):
    tok = {}
    for r in csv.DictReader(open(os.path.join(dd, 'robot-logs-1', 'clock_sync.csv'))):
        try:
            tok[int(r['token'].lstrip('c'))] = float(r['t_send'])
        except ValueError:
            pass
    ks = sorted(tok)
    t0 = tok[ks[0]]
    b = collections.defaultdict(list)
    for k in ks:
        b[int((tok[k] - t0) // 900)].append(k)
    xs, ys = [], []
    for i in sorted(b):
        v = b[i]
        n = max(v) - min(v) + 1
        if n < 200:
            continue
        xs.append(15 * i + 7.5)
        ys.append(100.0 * (n - len(v)) / n)
    return xs, ys
fig, ax = plt.subplots(figsize=(8, 3.4), dpi=130)
style(ax)
for dd, col, lab in ((common.D, C1, '21:14 (this session)'), (os.path.join(prev, 'e02-20261025-154522'), C2, '15:45'),
                     (os.path.join(prev, 'e02-20261025-151004'), C3, '15:10')):
    xs, ys = loss(dd)
    xs, ys = zip(*[(x, y) for x, y in zip(xs, ys) if x <= 110])
    ax.plot(xs, ys, color=col, lw=2, marker='o', ms=5, label=lab)
    ax.text(xs[-1] + 2, ys[-1], lab, fontsize=8, color=INK, va='center')
ax.set_xlim(0, 135)
ax.set_ylim(0, 14)
ax.set_xlabel('minutes since the session started', fontsize=8, color=INK)
ax.set_ylabel('clock pings lost (%)', fontsize=8, color=INK)
ax.legend(loc='upper right', fontsize=8, frameon=False)
fig.tight_layout()
fig.savefig(os.path.join(out, 'e02-oct25-2114-link.png'))

# 3. picture rate
P = list(csv.DictReader(open(os.path.join(common.D, 'perf.csv'))))
A, B = ep('21:14:00'), ep('22:57:00')
loc = lambda t: (t - 4 * 3600) % 86400 / 3600.0
mins = sorted(set(int(float(r['t']) // 60) for r in P if A <= float(r['t']) < B))
rx = [loc(m * 60 + 30) for m in mins]
ry = [S.median(float(r['fps_handled']) for r in P if int(float(r['t']) // 60) == m) for m in mins]
fts = []
for line in open(os.path.join(common.R, 'cam0/frames.csv'), 'rb').read().split(b'\n')[1:]:
    p = line.replace(b'\x00', b'').strip().split(b',')
    try:
        t = float(p[1])
    except (ValueError, IndexError):
        continue
    if A <= t < B:
        fts.append(t)
c = collections.Counter(int(t // 60) for t in fts)
cm = sorted(c)[1:-1]
fig, ax = plt.subplots(figsize=(9, 3.6), dpi=130)
style(ax)
h = lambda s: loc(ep(s))
for a, b, lab in (('21:17:23', '21:29:24', 'hold'), ('21:30:18', '21:36:53', 'hold'),
                  ('22:02:13', '22:47:13', 'three questions, 15 min each,\nnobody followed')):
    ax.axvspan(h(a), h(b), color=BAND, lw=0)
    ax.text((h(a) + h(b)) / 2, 1.0, lab, ha='center', va='bottom', fontsize=7.5, color=MUTED)
ax.plot([loc(m * 60 + 30) for m in cm], [c[m] / 60.0 for m in cm], color=C2, lw=2, label='recorder (camera 0)')
ax.plot(rx, ry, color=C1, lw=2, label='runner (tracker)')
for t, lab in (('21:16:00', 'explore starts'), ('21:37:24', 'rays'), ('22:47:15', 'parking, finds')):
    ax.axvline(h(t), color='#a3a29b', lw=0.8, ls=':')
    ax.text(h(t) + 0.005, 27.2, lab, fontsize=7.5, color=MUTED)
ax.set_ylim(0, 29)
ax.set_xlim(h('21:14:00'), h('22:57:00'))
tk = [('21:15', '21:15:00'), ('21:30', '21:30:00'), ('21:45', '21:45:00'), ('22:00', '22:00:00'), ('22:15', '22:15:00'),
      ('22:30', '22:30:00'), ('22:45', '22:45:00')]
ax.set_xticks([h(b) for _, b in tk])
ax.set_xticklabels([a for a, _ in tk])
ax.set_ylabel('pictures a second (median per minute)', fontsize=8, color=INK)
ax.legend(loc='center right', fontsize=8, frameon=False)
fig.tight_layout()
fig.savefig(os.path.join(out, 'e02-oct25-2114-speed.png'))
print('done')
