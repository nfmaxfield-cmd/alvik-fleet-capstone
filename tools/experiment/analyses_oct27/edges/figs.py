"""figs.py OUTDIR : the report's figures.
  e2-timeline.png : what each robot was doing in E2 (00:29:47-01:00:21), from e02_log.txt / e02_state.json times.
  edge-distances.png : the camera's distance from the open-edge line at every edge event, both stages (edgetable.py).
  heat-clock.png : tracker ms a picture against the Pi's real ARM clock, same workload, by heat flag (cpu.csv, perf.csv).
  side-pairs.png : where the side camera matched the overhead's robots tonight (sidepairs.py) on the board."""
import sys, os, csv, bisect
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from common import ep, rows, num, XMAX, YMAX, SCR
OUT = sys.argv[1]; os.makedirs(OUT, exist_ok=True)
ROB = {'Left': '#2a78d6', 'Center': '#eb6834', 'Right': '#1baf7a'}
INK, INK2, GRID = '#0b0b0b', '#52514e', '#e4e3df'
plt.rcParams.update({'font.size': 9, 'axes.edgecolor': INK2, 'axes.labelcolor': INK, 'xtick.color': INK2,
                     'ytick.color': INK2, 'axes.spines.top': False, 'axes.spines.right': False, 'figure.facecolor': '#fcfcfb',
                     'axes.facecolor': '#fcfcfb', 'savefig.facecolor': '#fcfcfb'})

# 1. E2 timeline -------------------------------------------------------------------------------------------------
S = {'work': ('#2a78d6', 'setting up, finding, driving a run'), 'own': ('#eb6834', 'held: waiting 3 min for an answer about itself'),
     'other': ('#eda100', 'stopped: another robot\'s hold'), 'still': ('#b9b8b2', 'standing still (naming check, waiting its turn)'),
     'out': ('#52514e', 'left out / put aside')}
T = lambda a: (ep(a) - ep('00:29:47')) / 60.0
spans = {
 'Center': [('00:29:58', '00:30:36', 'work'), ('00:30:36', '01:00:21', 'out')],
 'Left': [('00:29:58', '00:31:51', 'work'), ('00:31:51', '00:34:52', 'own'), ('00:34:52', '00:35:00', 'work'), ('00:35:00', '00:38:00', 'own'),
          ('00:38:00', '00:38:23', 'work'), ('00:38:23', '00:40:30', 'still'), ('00:40:30', '00:43:12', 'work'), ('00:43:12', '00:46:12', 'own'),
          ('00:46:12', '00:46:18', 'work'), ('00:46:18', '00:49:18', 'own'), ('00:49:18', '00:53:16', 'work'), ('00:53:16', '00:54:09', 'still'),
          ('00:54:09', '00:57:09', 'other'), ('00:57:09', '00:59:50', 'work'), ('00:59:50', '01:00:21', 'out')],
 'Right': [('00:29:58', '00:31:39', 'work'), ('00:31:51', '00:34:52', 'other'), ('00:35:00', '00:38:00', 'other'), ('00:38:23', '00:40:30', 'still'),
           ('00:40:30', '00:43:12', 'still'), ('00:43:12', '00:46:12', 'other'), ('00:46:18', '00:49:18', 'other'), ('00:49:18', '00:53:16', 'still'),
           ('00:53:16', '00:54:09', 'work'), ('00:54:09', '00:57:09', 'own'), ('00:57:11', '01:00:21', 'out')],
}
fig, ax = plt.subplots(figsize=(9, 2.9))
for i, r in enumerate(['Right', 'Left', 'Center']):
    for a, b, k in spans[r]:
        ax.barh(i, T(b) - T(a), left=T(a), height=0.6, color=S[k][0], edgecolor='#fcfcfb', linewidth=1)
ax.set_yticks(range(3)); ax.set_yticklabels(['Right', 'Left', 'Center'])
ev = [('00:30:36', 'Center: turns fail\n(hanging from E1)'), ('00:57:11', 'Right: rescue\nfails'), ('00:59:50', 'Left: at the\nedge, aside')]
for a, txt in ev:
    ax.axvline(T(a), color=INK2, lw=0.6, ls=':')
ax.set_xlim(0, T('01:00:21') + 0.2); ax.set_xlabel('minutes into E2 (00:29:47)')
ticks = ['00:30', '00:35', '00:40', '00:45', '00:50', '00:55', '01:00']
ax.set_xticks([T(t + ':00') for t in ticks]); ax.set_xticklabels(ticks)
ax.set_title('E2: what each robot was doing, minute by minute (data: E2 e02_log.txt and e02_state.json)', loc='left', fontsize=9, color=INK)
h = [Rectangle((0, 0), 1, 1, color=c) for c, _ in S.values()]
ax.legend(h, [l for _, l in S.values()], loc='upper center', bbox_to_anchor=(0.5, -0.32), ncol=3, frameon=False, fontsize=8)
fig.tight_layout(); fig.savefig(OUT + '/e2-timeline.png', dpi=150); plt.close(fig)

# 2. edge distances ----------------------------------------------------------------------------------------------
E = list(csv.DictReader(open(SCR + '/edge_events.csv')))
cats = [('own edge stop (line', 'own stop: line sensors'), ('own edge stop (bottom', 'own stop: bottom sensor'),
        ('camera stop: border', 'camera border hold'), ('camera stop', 'camera watchdog stop')]
fig, ax = plt.subplots(figsize=(8, 3.0))
for j, (key, lab) in enumerate(cats):
    for r in E:
        k = next((c for c, _ in cats if r['kind'].startswith(c)), None)
        if k != key or not r['cam_cm']: continue
        y = j + (-0.12 if r['stage'] == 'E1' else 0.12)
        ax.scatter(float(r['cam_cm']), y, s=42, color=ROB[r['robot']], marker='o' if r['stage'] == 'E2' else 's',
                   edgecolor='#fcfcfb', linewidth=1.5, zorder=3)
ax.axvline(0, color=INK, lw=1); ax.text(0.2, 3.45, 'open-edge line (camera)', fontsize=7.5, color=INK2)
ax.axvline(9, color=INK2, lw=0.7, ls='--'); ax.text(9.2, 3.45, 'no turns nearer (9 cm)', fontsize=7.5, color=INK2)
ax.axvline(15, color=INK2, lw=0.7, ls=':'); ax.text(15.2, 3.45, 'camera border (15 cm)', fontsize=7.5, color=INK2)
ax.set_yticks(range(4)); ax.set_yticklabels([l for _, l in cats]); ax.set_ylim(-0.5, 3.7)
ax.set_xlabel("robot's centre from the nearest open-edge line, by the overhead camera at the moment (cm; negative = past it)")
ax.grid(axis='x', color=GRID, lw=0.6)
for r, c in ROB.items(): ax.scatter([], [], color=c, s=30, label=r)
ax.scatter([], [], color='#b9b8b2', marker='s', s=30, label='E1'); ax.scatter([], [], color='#b9b8b2', marker='o', s=30, label='E2')
ax.legend(frameon=False, fontsize=8, ncol=5, loc='lower right', bbox_to_anchor=(1, 1.05))
ax.set_title('Every edge event (data: robot-logs edge.csv, e02_state.json, track.csv; both stages)', loc='left', fontsize=9, y=1.13)
fig.tight_layout(); fig.savefig(OUT + '/edge-distances.png', dpi=150); plt.close(fig)

# 3. heat vs clock -----------------------------------------------------------------------------------------------
BANDS = [(1790, 1900, '1800 (full)'), (1650, 1790, '1677-1775'), (1500, 1650, '1531-1630'), (0, 1000, '600 (throttled)')]
data = [[] for _ in BANDS]
for st in ('E1', 'E2'):
    p = rows(st, 'perf.csv'); pt = [float(r['t']) for r in p]
    for r in rows(st, 'cpu.csv'):
        m = num(r['arm_mhz']); i = bisect.bisect_left(pt, float(r['t']))
        if i >= len(p) or m is None: continue
        q = p[i]
        if q['phase'] != 'parking' or not q['source'] or num(q['ms_mean']) is None: continue
        for j, (lo, hi, _) in enumerate(BANDS):
            if lo <= m < hi: data[j].append(num(q['ms_mean']))
fig, ax = plt.subplots(figsize=(6.4, 3.2))
bp = ax.boxplot(data, widths=0.5, patch_artist=True, showfliers=False, medianprops=dict(color=INK, lw=1.5))
for b_, c in zip(bp['boxes'], ['#2a78d6', '#eda100', '#eb6834', '#e34948']): b_.set_facecolor(c); b_.set_alpha(0.75)
for j, d in enumerate(data):
    ax.text(j + 1, max(d) + 2 if d else 0, 'n = %d' % len(d), ha='center', fontsize=7.5, color=INK2)
ax.set_xticks(range(1, 5)); ax.set_xticklabels([l for _, _, l in BANDS])
ax.set_xlabel("the Pi's real CPU clock at that moment (MHz, cpu.csv arm_mhz)"); ax.set_ylabel('tracker ms a picture')
ax.grid(axis='y', color=GRID, lw=0.6)
ax.set_title('Same workload (one robot followed, the others parked), both stages\n(data: cpu.csv every 2 s joined to perf.csv)', loc='left', fontsize=9)
fig.tight_layout(); fig.savefig(OUT + '/heat-clock.png', dpi=150); plt.close(fig)
import statistics as sx
print('heat medians', [(l, len(d), sx.median(d)) for (_, _, l), d in zip(BANDS, data) if d])
print('ok')
