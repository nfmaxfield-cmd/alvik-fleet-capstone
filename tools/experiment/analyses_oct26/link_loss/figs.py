"""figs.py OUTDIR : the link figure of the 2026-10-26 runner-fix report, from linkloss.json (linkloss.py).
  e02-oct26-link-windows.png  every real session's command loss per 5-minute window (clock pings no robot answered),
                              windows with a hold marked, against the warn (5 %) and pause (15 %) levels."""
import json, os, sys
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
out = sys.argv[1]
HERE = os.path.dirname(os.path.abspath(__file__))
R = json.load(open(os.path.join(HERE, 'linkloss.json')))
INK, MUTED, GRID = '#3d3c38', '#6b6a64', '#e4e3df'
C1, C2, WARN, PAUSE = '#2a78d6', '#eb6834', '#c99a06', '#d03b2f'
fig, ax = plt.subplots(figsize=(11, 4.2), dpi=130)
for s in ax.spines.values():
    s.set_visible(False)
ax.grid(axis='y', color=GRID, lw=0.8); ax.set_axisbelow(True)
ax.tick_params(labelsize=8, colors=MUTED, length=0)
labels = []
for i, r in enumerate(R):
    labels.append('Oct %s %s:%s' % (r['session'][10:12], r['session'][13:15], r['session'][15:17]))
    act = [w for w in r['windows'] if w['sends'] > 0]           # windows while commands were being sent
    idle = [w for w in r['windows'] if w['sends'] == 0]
    jit = lambda k, n: (k % 7 - 3) * 0.06
    ax.scatter([i + jit(k, 0) for k, w in enumerate(idle)], [100 * w['loss'] for w in idle], s=10, color='#b9b7b0',
               zorder=2, label='5-min window, no commands sent' if i == 0 else None)
    ax.scatter([i + jit(k, 0) for k, w in enumerate(act)], [100 * w['loss'] for w in act], s=16, color=C1, zorder=3,
               label='5-min window, commands sent' if i == 0 else None)
    hw = [w for w in act if w['holds'] > 0]
    ax.scatter([i + jit(k, 0) for k, w in enumerate(act) if w['holds'] > 0], [100 * w['loss'] for w in hw], s=60,
               facecolors='none', edgecolors=C2, lw=1.4, zorder=4, label='window with a hold' if not i else None)
ax.axhline(5, color=WARN, ls='--', lw=1.2); ax.axhline(15, color=PAUSE, ls='--', lw=1.2)
ax.text(-0.4, 5.6, 'warn 5 %', color=WARN, fontsize=8, ha='left')
ax.text(-0.4, 15.6, 'pause 15 %', color=PAUSE, fontsize=8, ha='left')
ax.set_xticks(range(len(R))); ax.set_xticklabels(labels, fontsize=7.5, color=MUTED, rotation=30, ha='right')
ax.set_ylim(-1, 35)
ax.set_ylabel('clock pings lost, % (5-min window)', fontsize=8.5, color=INK)
ax.set_title('Radio link loss in every real session (21:14 overnight windows above 35 % not shown)', fontsize=10,
             color=INK, loc='left')
ax.legend(fontsize=7.5, frameon=False, loc='upper center')
fig.text(0.01, 0.01, 'Measured: share of the runner\'s clock pings no robot answered, per 5 minutes. Data: each session\'s '
         'robot-logs/clock_sync.csv and e02_state.json holds (GitHub releases e02-session-*, Oct 22-25).', fontsize=7,
         color=MUTED)
fig.tight_layout(rect=(0, 0.04, 1, 1))
fig.savefig(os.path.join(out, 'e02-oct26-link-windows.png'))
print('->', out)
