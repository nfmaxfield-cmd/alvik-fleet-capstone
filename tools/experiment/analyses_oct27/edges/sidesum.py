"""sidesum.py PAIRS.csv OUT.png : tonight's side-camera matches (miss under 20 px) by stage, robot and edge (within
15 cm of each open-edge line by the overhead), and a map of where they were on the board."""
import sys, csv, statistics as sx
from collections import Counter
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from common import XMAX, YMAX
R = list(csv.DictReader(open(sys.argv[1])))
tot = len(R); m = [r for r in R if r['miss_px'] and float(r['miss_px']) < 20]
print('robot moments (overhead spot at a side picture):', tot, ' matched:', len(m), '(%.0f %%)' % (100 * len(m) / tot))
print('median miss px (matched):', sx.median(float(r['miss_px']) for r in m))
print('by stage', Counter(r['stage'] for r in m), 'by robot', Counter(r['robot'] for r in m))
for side, f in (('x=0', lambda x, y: x), ('x=122', lambda x, y: XMAX - x), ('y=0', lambda x, y: y), ('y=122', lambda x, y: YMAX - y)):
    near = [r for r in R if f(float(r['x']), float(r['y'])) < 15]
    mm = [r for r in near if r['miss_px'] and float(r['miss_px']) < 20]
    print('within 15 cm of %-6s: %5d moments, %5d matched (%.0f %%), robots %s' % (side, len(near), len(mm), 100 * len(mm) / max(1, len(near)), dict(Counter(r['robot'] for r in mm))))
fig, ax = plt.subplots(figsize=(4.6, 4.4))
ax.add_patch(Rectangle((0, 0), XMAX, YMAX, fill=False, ec='#0b0b0b', lw=1))
ax.add_patch(Rectangle((107, 0), YMAX - 107, XMAX, color='#e34948', alpha=0.12, lw=0))   # (y, x): the far strip y > 107
ax.add_patch(Rectangle((0, 115), 30, XMAX - 115, color='#e34948', alpha=0.12, lw=0))     # x > 115 with y < 30
un = [r for r in R if not (r['miss_px'] and float(r['miss_px']) < 20)]
ax.scatter([float(r['y']) for r in m], [float(r['x']) for r in m], s=4, color='#2a78d6', label='matched by the side camera', lw=0)
ax.scatter([float(r['y']) for r in un], [float(r['x']) for r in un], s=4, color='#b9b8b2', label='overhead spot, no side match', lw=0)
ax.set_xlim(YMAX + 3, -3); ax.set_ylim(-3, XMAX + 3); ax.set_aspect('equal')
ax.set_xlabel('y (cm)'); ax.set_ylabel('x (cm)')
ax.text(114.5, 88, 'side camera cannot see\n(earlier study)', ha='center', va='center', rotation=90, fontsize=7, color='#52514e')
ax.legend(frameon=False, fontsize=7.5, loc='upper center', bbox_to_anchor=(0.5, -0.13), ncol=1, markerscale=3)
ax.set_title('Where tonight\'s side pictures matched the overhead\n(data: cam1 video every 0.4 s, track.csv, both stages)', fontsize=9, loc='left')
for s in ('top', 'right'): ax.spines[s].set_visible(False)
fig.tight_layout(); fig.savefig(sys.argv[2], dpi=150)
