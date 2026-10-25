"""The runner's pictures handled a second against the recorder's, per minute, 2026-10-25 15:45 session."""
import sys, os, csv, time, statistics as S
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
D = sys.argv[1]; R = sys.argv[2]; out = sys.argv[3]
P = list(csv.DictReader(open(os.path.join(D, 'perf.csv'))))
fr = []
for line in open(os.path.join(R, 'cam0/frames.csv'), 'rb').read().split(b'\n')[1:]:
    p = line.replace(b'\x00', b'').strip().split(b',')
    try: fr.append(float(p[1]))
    except (ValueError, IndexError): pass
loc = lambda t: (t - 4 * 3600) % 86400 / 3600.0
mins = sorted(set(int(float(r['t']) // 60) for r in P))
rx = [loc(m * 60 + 30) for m in mins]
ry = [S.median(float(r['fps_handled']) for r in P if int(float(r['t']) // 60) == m) for m in mins]
import collections
c = collections.Counter(int(t // 60) for t in fr)
cm = sorted(k for k in c if c[k] > 0)[1:-2]
cx = [loc(m * 60 + 30) for m in cm]; cy = [c[m] / 60.0 for m in cm]
fig, ax = plt.subplots(figsize=(9, 3.6), dpi=130)
for s in ax.spines.values(): s.set_visible(False)
ax.grid(axis='y', color='#e4e3df', lw=0.8); ax.set_axisbelow(True)
h = lambda s: int(s[:2]) + int(s[3:5]) / 60 + int(s[6:8]) / 3600
for a, b, lab in (('15:45:23', '15:56:00', 'outline question\n(tracker idle)'), ('16:29:31', '16:31:37', 'naming\ncheck')):
    ax.axvspan(h(a), h(b), color='#efeeea', lw=0)
    ax.text((h(a) + h(b)) / 2, 17, lab, ha='center', va='bottom', fontsize=7.5, color='#6b6a64')
ax.plot(cx, cy, color='#eb6834', lw=2, label='recorder (camera 0)')
ax.plot(rx, ry, color='#2a78d6', lw=2, label='runner (tracker)')
ax.text(cx[-1] + 0.01, cy[-1], 'recorder', color='#3d3c38', fontsize=8, va='center')
ax.text(rx[-1] + 0.01, ry[-1], 'runner', color='#3d3c38', fontsize=8, va='center')
for t, lab in (('15:56:05', 'tracker on'), ('15:58:46', 'robots found'), ('15:59:34', 'first robot driven')):
    ax.axvline(h(t), color='#a3a29b', lw=0.8, ls=':')
ax.text(h('15:56:05'), 26.3, 'tracker on', fontsize=7.5, color='#6b6a64', ha='right')
ax.text(h('15:59:34'), 26.3, 'robots found, then driven', fontsize=7.5, color='#6b6a64', ha='left')
ax.set_ylim(0, 28); ax.set_xlim(h('15:45:00'), h('17:31:00'))
ticks = [h('%02d:%02d:00' % (hh, mm)) for hh, mm in ((15, 45), (16, 0), (16, 15), (16, 30), (16, 45), (17, 0), (17, 15), (17, 30))]
ax.set_xticks(ticks); ax.set_xticklabels(['15:45', '16:00', '16:15', '16:30', '16:45', '17:00', '17:15', '17:30'], fontsize=8, color='#6b6a64')
ax.tick_params(axis='y', labelsize=8, colors='#6b6a64', length=0); ax.tick_params(axis='x', length=0)
ax.set_ylabel('pictures a second (median per minute)', fontsize=8, color='#3d3c38')
ax.legend(loc='lower right', fontsize=8, frameon=False, bbox_to_anchor=(0.93, 0.02))
fig.tight_layout(); fig.savefig(out)
