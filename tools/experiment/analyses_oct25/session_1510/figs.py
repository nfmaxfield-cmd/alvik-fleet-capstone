import sys, csv, json, math
exec(open('/tmp/claude-0/s1025/work/trk.py').read().split("if __name__")[0])
import numpy as np, cv2, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT = sys.argv[1] if len(sys.argv) > 1 else '/tmp/claude-0/s1025/fr'
col = {'Left': '#2a78d6', 'Center': '#eb6834', 'Right': '#1baf7a'}
# ---- figure 1: what explore covered
img = cv2.cvtColor(cv2.imread('/tmp/claude-0/s1025/fr/f_153200.png'), cv2.COLOR_BGR2RGB)
fig, ax = plt.subplots(figsize=(8, 6.2), dpi=110)
ax.imshow(img, alpha=0.55)
W, H = 122.15, 121.9
C = [(0, 0), (W, 0), (W, H), (0, H), (0, 0)]
pc = np.array([np.array(guard.project((x, y, 0.0))).ravel() for x, y in C])
ax.plot(pc[:, 0], pc[:, 1], color='#52514e', lw=1.2, ls='--', label='stored calibration outline (roam1)')
for r in ('Left', 'Center', 'Right'):
    q = [p for p in rows if p['r'] == r and p['ph'] == 'finding']
    seg, first = [q[0]], True
    for a, b in zip(q, q[1:] + [None]):
        if b is None or b['t'] - a['t'] > 2.0 or math.hypot(b['u'] - a['u'], b['v'] - a['v']) > 15:
            ax.plot([p['u'] for p in seg], [p['v'] for p in seg], color=col[r], lw=1.0, label=('%s path (camera)' % r) if first else None)
            first = False; seg = [b] if b else []
        else:
            seg.append(b)
d = json.load(open(L + 'e02_state.json'))
for r, u, v, k in d['setup']['explore_live']['contacts']:
    ax.scatter([u], [v], s=70, marker='o' if k == 'board' else 'X', facecolor=col[r], edgecolor='white', linewidth=1.5, zorder=5)
ax.scatter([], [], s=70, marker='o', color='#52514e', label='ray end: "board"')
ax.scatter([], [], s=70, marker='X', color='#52514e', label='ray end: open edge')
ax.text(250, 32, 'top board: no ray reached it\n(nearest robot centre 16 cm away)', ha='center', fontsize=9, color='#0b0b0b',
        bbox=dict(fc='white', ec='none', alpha=0.8))
ax.text(268, 352, 'object past the open edge:\nread as "board" twice', ha='center', fontsize=8.5, color='#0b0b0b',
        bbox=dict(fc='white', ec='none', alpha=0.8))
ax.set_xlim(60, 500); ax.set_ylim(400, 0)
ax.set_xlabel('picture x (pixels)'); ax.set_ylabel('picture y (pixels)')
ax.set_title('Explore, 15:10-15:26: where the robots drove and what their rays met', fontsize=11)
ax.legend(loc='lower right', fontsize=7.5, framealpha=0.9)
fig.tight_layout(); fig.savefig(OUT + '/e02-oct25-1510-explore.png'); plt.close(fig)
# ---- figure 2: camera speed against what the runner was doing
perf = [(float(r['t']), float(r['fps_handled'])) for r in csv.DictReader(open(L + 'perf.csv'))]
base = 1792955404 - (15 * 3600 + 10 * 60 + 4)
m = lambda t: (t - base) / 60.0 - 15 * 60          # minutes past 15:00
fig, ax = plt.subplots(figsize=(8, 3.2), dpi=110)
t = np.array([m(a) for a, _ in perf]); f = np.array([b for _, b in perf])
ok = t < 26.75
ax.plot(t[ok], f[ok], color='#2a78d6', lw=1.2)
ax.axhline(12.5, color='#52514e', lw=0.8, ls=':')
ax.text(10.2, 12.9, 'slow line (12.5)', fontsize=8, color='#52514e')
P = lambda h, mi, s: base + h * 3600 + mi * 60 + s
spans = [((15, 24, 38), (15, 25, 3), 'explore\nfit 1'), ((15, 26, 6), (15, 26, 34), 'fit\n2'),
         ((15, 21, 9), (15, 21, 17), 'lock lost:\nwhole-picture search')]
for a, b, lab in spans:
    ax.axvspan(m(P(*a)), m(P(*b)), color='#eda100', alpha=0.25, lw=0)
    ax.text(m(P(*a)), 3.0, lab, fontsize=7.5, color='#0b0b0b')
for h in d['holds']:
    ax.axvspan(m(h[0]), m(h[0] + h[6]), color='#52514e', alpha=0.12, lw=0)
ax.text(14.3, 27, 'grey = holds (waiting for Nick)', fontsize=7.5, color='#52514e')
ax.set_xlim(10, 26.8); ax.set_ylim(0, 30)
ax.set_xlabel('time (minutes past 15:00, 2026-10-25)'); ax.set_ylabel('pictures handled a second')
ax.set_title('Runner camera speed through explore (perf.csv)', fontsize=11)
fig.tight_layout(); fig.savefig(OUT + '/e02-oct25-1510-speed.png'); plt.close(fig)
print('ok')
