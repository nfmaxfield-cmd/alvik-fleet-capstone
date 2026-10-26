"""figs.py OUTDIR : the report's figures.
 1 cost.png    : the tracker's time per picture (perf.csv ms_mean, each second) by what it was doing, coloured by the
                 Pi's heat flag at that moment (cpu.csv): workload against heat.
 2 edge.png    : overhead video crops of Center going over the right edge (10:06-10:10) and Left's rescue find
                 ending beside it (11:05:49), with the stored open-edge line (red) and 4 cm outside it (blue).
 3 grid.png    : explore's own camera against the stored roam1 calibration: a 20 cm grid on the board by roam1
                 (white) and the same grid by explore's camera after the best rotation + shift (orange).
 4 contrast.png: the three robots at 09:56:41 (Left washed out) and Left at 10:01:08, with the darkest-pixel numbers."""
import sys, os, bisect, json, calendar
import numpy as np, cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pose
import e02_explore as EX
from common import D, W, rows, ep, lt
OUT = sys.argv[1]
os.makedirs(OUT, exist_ok=True)
BLUE, ORANGE, AQUA = '#2a78d6', '#eb6834', '#1baf7a'
INK, INK2, SURF = '#0b0b0b', '#52514e', '#fcfcfb'
plt.rcParams.update({'font.size': 10, 'axes.edgecolor': '#b5b4ad', 'axes.labelcolor': INK2, 'xtick.color': INK2,
                     'ytick.color': INK2, 'figure.facecolor': SURF, 'axes.facecolor': SURF})

# ---- 1 cost by workload ------------------------------------------------------------------------------------------
cpu = [(float(r['t']), int(r['throttled'], 16)) for r in rows('cpu.csv')]
ct = [c[0] for c in cpu]
def hot(t):
    i = min(max(bisect.bisect_left(ct, t), 0), len(ct) - 1)
    return bool(cpu[i][1] & 0b1110)
W_ = [('Nothing followed\nor watched', [('09:55:44', '09:56:31'), ('10:37:20', '10:47:20')]),
      ('Parked robots\nwatched only', [('10:22:15', '10:36:55'), ('10:50:00', '10:54:30'), ('10:55:10', '11:04:55')]),
      ('One robot followed,\nparked ones watched', [('10:03:45', '10:21:55')])]
perf = [(float(r['t']), float(r['ms_mean'])) for r in rows('perf.csv')]
fig, ax = plt.subplots(figsize=(8.4, 4.4))
rng = np.random.default_rng(1)
summary = []
for k, (name, spans) in enumerate(W_):
    pts = [(t, ms) for t, ms in perf if any(ep(a) <= t <= ep(b) for a, b in spans)]
    for flag, col, lab in ((False, BLUE, 'Pi not throttling (no heat flag)'), (True, ORANGE, 'Pi hot: soft limit or throttling flag on')):
        y = np.array([ms for t, ms in pts if hot(t) == flag])
        if len(y) == 0:
            continue
        x = k + (0.18 if flag else -0.18) + rng.uniform(-0.12, 0.12, len(y))
        ax.scatter(x, y, s=9, color=col, alpha=0.55, linewidths=0, label=lab if (k == 0) else None)
        med = float(np.median(y))
        ax.plot([k + (0.18 if flag else -0.18) - 0.15, k + (0.18 if flag else -0.18) + 0.15], [med, med], color=INK, lw=2)
        ax.text(k + (0.18 if flag else -0.18) + 0.17, med, '%.0f ms' % med, va='center', fontsize=9, color=INK)
        summary.append((name.replace('\n', ' '), flag, len(y), med, float(np.percentile(y, 95))))
ax.axhline(40, color=INK2, lw=1, ls='--')
ax.text(-0.55, 42, 'over 40 ms a picture: under 25 pictures a second', ha='left', fontsize=8.5, color=INK2)
ax.set_xticks(range(3)); ax.set_xticklabels([n for n, _ in W_])
ax.set_ylabel('Tracker time per picture (ms)')
ax.set_ylim(0, 160); ax.set_xlim(-0.6, 2.6)
ax.grid(axis='y', color='#e6e5e0', lw=0.8); ax.set_axisbelow(True)
for s in ('top', 'right'):
    ax.spines[s].set_visible(False)
ax.legend(loc='upper left', frameon=False, fontsize=9)
ax.set_title('What slowed the camera: the work per picture, not the heat', loc='left', fontsize=11, color=INK)
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'cost.png'), dpi=130); plt.close(fig)
for s in summary:
    print('%-45s hot=%s n=%4d median %.1f ms p95 %.1f' % s)

# ---- 2 Center over the right edge ------------------------------------------------------------------------------
FR = W + '/fr'
fr = sorted((float(t), int(k)) for k, t in (l.strip().split(',') for l in open(os.path.join(FR, 'times.csv'))))
fts = [f[0] for f in fr]
def frame(s):
    i = min(range(len(fts)), key=lambda j: abs(fts[j] - ep(s)))
    return cv2.imread(os.path.join(FR, '%06d.jpg' % fr[i][1])), lt(fts[i])[:8]
def lines(im):
    for off, col in ((0, (0, 0, 220)), (-4, (214, 120, 42))):
        p = [tuple(int(round(c)) for c in pose.board_px(off if ax_ == 'x' else a, a if ax_ == 'x' else off))
             for ax_ in ('y',) for a in np.linspace(-10, 125, 40)]
        for a, b in zip(p, p[1:]):
            cv2.line(im, a, b, col, 1, cv2.LINE_AA)
    return im
tiles = []
u0, v0 = pose.board_px(38, -1, 3.7)
H, Z = 42, 4
for s, cap in (('10:06:42.9', 'edge stop 1;\nback-out held'), ('10:08:44.0', 'edge stop 2;\nback-out held'),
               ('10:09:11.9', 'ray 6 starts\nat the line'), ('10:09:41.0', 'wheels 105 cm,\nbody over the drop'),
               ('11:05:49.4', 'Left beside it,\nnose over too')):
    im, tl = frame(s)
    im = lines(im)
    c = im[int(v0 - H):int(v0 + H), int(u0 - H):int(u0 + H)]
    c = cv2.resize(c, None, fx=Z, fy=Z, interpolation=cv2.INTER_CUBIC)
    c = cv2.copyMakeBorder(c, 52, 0, 0, 4, cv2.BORDER_CONSTANT, value=(251, 252, 252))
    cv2.putText(c, tl, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (11, 11, 11), 1, cv2.LINE_AA)
    for j, ln in enumerate(cap.split('\n')):
        cv2.putText(c, ln, (6, 34 + 14 * j), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (78, 81, 82), 1, cv2.LINE_AA)
    tiles.append(c)
cv2.imwrite(os.path.join(OUT, 'edge.png'), np.hstack(tiles))

# ---- 3 explore's camera against roam1 --------------------------------------------------------------------------
import fit_replay as FRP  # noqa: E402  (runs the replay; prints its lines)
r, sol = FRP.run('as run', dict(FRP.obs))
corners = [pose.board_px(x, y, 0) for x, y in ((0, 0), (122.15, 0), (122.15, 121.9), (0, 121.9))]
poly = np.array(corners, np.float32)
A, B = [], []
for u in range(0, 640, 10):
    for v in range(0, 480, 10):
        if cv2.pointPolygonTest(poly, (u, v), False) > 0:
            A.append(EX.world_of(sol, u, v, 0.0)); q = pose.ROAM.to_floor(float(u), float(v), 0.0)
            B.append((float(q[0]), float(q[1])))
A, B = np.array(A), np.array(B)
ma, mb = A.mean(0), B.mean(0)
U_, S_, Vt = np.linalg.svd((A - ma).T @ (B - mb)); R = Vt.T @ U_.T
im, tl = frame('10:47:20')
im = cv2.addWeighted(im, 0.75, np.full_like(im, 255), 0.25, 0)
def roam_px(x, y):
    return tuple(int(round(c)) for c in pose.board_px(x, y, 0))
def expl_px(x, y):
    a = R.T @ (np.array([x, y]) - mb) + ma            # roam1 cm -> explore's own cm (rigid)
    uv = EX.project(np.array([[a[0], a[1], 0.0]]), sol['f'], sol['k1'], np.array(sol['rvec']), np.array(sol['tvec']), (640, 480))[0]
    return int(round(uv[0])), int(round(uv[1]))
for fn, col, wdt in ((roam_px, (255, 255, 255), 2), (expl_px, (52, 104, 235), 2)):
    for g in np.arange(0, 122.2, 20.0).tolist() + [122.15]:
        for pts in ([(g, y) for y in np.linspace(0, 121.9, 25)], [(x, g) for x in np.linspace(0, 122.15, 25)]):
            p = [fn(x, y) for x, y in pts]
            for a, b in zip(p, p[1:]):
                cv2.line(im, a, b, col, wdt if g in (0, 122.15, 121.9) else 1, cv2.LINE_AA)
im = im[0:420, 60:520]
im = cv2.copyMakeBorder(im, 30, 0, 0, 0, cv2.BORDER_CONSTANT, value=(251, 252, 252))
cv2.putText(im, 'white: stored calibration (roam1)', (6, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (11, 11, 11), 1, cv2.LINE_AA)
cv2.putText(im, "orange: explore's own camera, 20 cm grid", (6, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (52, 104, 235), 1, cv2.LINE_AA)
cv2.imwrite(os.path.join(OUT, 'grid.png'), cv2.resize(im, None, fx=1.6, fy=1.6, interpolation=cv2.INTER_CUBIC))

# ---- 4 contrast --------------------------------------------------------------------------------------------------
still = cv2.imread(D + '/frames/stills/095641_hold-Left.jpg')
g = cv2.cvtColor(still, cv2.COLOR_BGR2GRAY).astype(float)
tiles = []
def tile(img, gray, u, v, label):
    ring = np.concatenate([gray[v - 28:v - 18, u - 28:u + 28].ravel(), gray[v + 18:v + 28, u - 28:u + 28].ravel()])
    body = gray[v - 10:v + 11, u - 10:u + 11]
    bm, p5 = np.median(ring), np.percentile(body, 5)
    c = cv2.resize(img[v - 26:v + 26, u - 26:u + 26], None, fx=5, fy=5, interpolation=cv2.INTER_CUBIC)
    c = cv2.copyMakeBorder(c, 52, 0, 0, 4, cv2.BORDER_CONSTANT, value=(251, 252, 252))
    cv2.putText(c, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (11, 11, 11), 1, cv2.LINE_AA)
    cv2.putText(c, 'board %.0f, darkest %.0f: gap %.0f' % (bm, p5, bm - p5), (6, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.47,
                (78, 81, 82), 1, cv2.LINE_AA)
    print(label, 'board %.0f darkest(p5) %.0f gap %.0f' % (bm, p5, bm - p5))
    return c
for n, (u, v) in (('Left 09:56:41', (205, 130)), ('Center 09:56:41', (242, 218)), ('Right 09:56:41', (340, 235))):
    tiles.append(tile(still, g, u, v, n))
im2, _ = frame('10:01:08')
tiles.append(tile(im2, cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY).astype(float), 197, 133, 'Left 10:01:08 (turned)'))
cv2.imwrite(os.path.join(OUT, 'contrast.png'), np.hstack(tiles))
print('written', OUT)
