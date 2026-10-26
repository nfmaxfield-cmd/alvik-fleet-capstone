import sys, os, pickle, json, csv; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common2 import *
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import sess, vid
OUT = '../figs/'
C = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']
plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False, 'axes.edgecolor': '#52514e', 'axes.labelcolor': '#0b0b0b', 'xtick.color': '#52514e', 'ytick.color': '#52514e', 'axes.grid': True, 'grid.color': '#e6e5e0', 'grid.linewidth': 0.6, 'figure.facecolor': '#fcfcfb', 'axes.facecolor': '#fcfcfb'})
d = pickle.load(open('../work/pooled.pkl', 'rb')); m = d['m']
R1 = CAM.load_camera(REPO + '/tools/experiment/data/side_floorcal_roam1.json')
W, L = CAM.BOARD_W_CM, CAM.BOARD_L_CM
# 1. view and coverage
o = cv2.imread('../work/med_0955_cam0.jpg'); sdm = cv2.imread('../work/med_0955_cam1.jpg')
xs = np.arange(0, W, 1.0); ys = np.arange(0, L, 1.0)
ov = o.copy()
for x in xs:
    for y in ys:
        u, v = R1.project([x, y, 1.85]); vis = 20 <= u <= 620 and 20 <= v <= 460
        ou, ov_ = OCAM.project([x, y, 0])
        if not vis: cv2.circle(ov, (int(ou), int(ov_)), 2, (40, 40, 220), -1)
o2 = cv2.addWeighted(o, 0.5, ov, 0.5, 0)
edge = [(0, 0), (W, 0), (W, L), (0, L), (0, 0)]
def outline(cam, img, col, z):
    pts = []
    for (a, b), (c, e) in zip(edge[:-1], edge[1:]):
        for s_ in np.linspace(0, 1, 60): pts.append(cam.project([a + (c - a) * s_, b + (e - b) * s_, z]))
    pts = np.array(pts); ok = np.all(np.isfinite(pts), axis=1); cv2.polylines(img, [pts[ok].astype(np.int32)], False, col, 2)
outline(OCAM, o2, (0, 200, 255), 0); outline(R1, sdm, (0, 200, 255), 0)
for name, cam in [('side', R1)]:
    for (x, y, lab) in [(0, 0, '(0,0)'), (W, 0, '(122,0)'), (0, 100, '(0,100)'), (W, 100, '(122,100)')]:
        u, v = cam.project([x, y, 0]); cv2.putText(sdm, lab, (int(u) + 4, int(v) - 4), 0, 0.45, (0, 255, 255), 1)
        u, v = OCAM.project([x, y, 0]); cv2.putText(o2, lab, (int(u) + 4, int(v) - 4), 0, 0.45, (0, 255, 255), 1)
cv2.imwrite(OUT + 'twocam-side-view-coverage.png', np.hstack([o2, sdm]))
# 2. spots needed
r = pickle.load(open('../work/spread.pkl', 'rb'))['res']
fig, ax = plt.subplots(figsize=(7.5, 4.2))
for N, c in zip([4, 6, 10, 20], C):
    k = r[:, 0] == N
    ax.scatter(r[k, 1] * 100, np.clip(r[k, 2], 0, 10), s=12, color=c, alpha=0.6, label='%d spots' % N, edgecolors='none')
ax.set_xlabel('How much of the board the spots span (% of board area, convex hull)'); ax.set_ylabel('Disagreement with the overhead (cm, median)')
ax.set_ylim(0, 10.3); ax.legend(frameon=False, loc='upper right'); ax.set_title('Robot spots needed to calibrate the side camera', loc='left')
ax.axhline(1.5, color='#52514e', lw=1, ls='--'); ax.text(78, 1.65, 'all 115 spots: 1.5 cm', color='#52514e', fontsize=9)
fig.tight_layout(); fig.savefig(OUT + 'twocam-spots-needed.png', dpi=130); plt.close(fig)
# 3. agreement map + precision shape
reg = json.load(open('../work/regions.json'))
fig, axs = plt.subplots(1, 2, figsize=(10, 4.6))
ax = axs[0]
names = [['near left', 'near centre', 'near right'], ['middle left', 'middle centre', 'middle right'], ['far left', 'far centre', 'far right']]
M = np.array([[reg[n]['med'] for n in row] for row in names])
im_ = ax.imshow(M, origin='lower', extent=[0, 120, 0, 120], cmap=matplotlib.colors.LinearSegmentedColormap.from_list('b', ['#cde2fb', '#2a78d6', '#104281']), vmin=0, vmax=4)
for i, row in enumerate(names):
    for j, n in enumerate(row):
        ax.text(20 + 40 * j, 20 + 40 * i, '%.1f cm\n95%%: %.1f\nn=%d' % (reg[n]['med'], reg[n]['p95'], reg[n]['n']), ha='center', va='center', fontsize=9, color='#0b0b0b' if reg[n]['med'] < 2.2 else '#fcfcfb')
ax.set_xlabel('x (cm)'); ax.set_ylabel('y (cm): side camera below, at y = -54'); ax.set_title('Where both see a robot: how far apart', loc='left'); ax.grid(False)
fig.colorbar(im_, ax=ax, fraction=0.046, label='median cm')
ax = axs[1]; sig = 2.0  # px, blob-centre noise taken for the drawing
for x in range(10, 121, 20):
    for y in range(10, 111, 20):
        J = m.jac(x, y); Cv = np.linalg.inv(J.T @ J) * sig ** 2; w, V = np.linalg.eigh(Cv)
        ang = np.degrees(np.arctan2(V[1, 1], V[0, 1]))
        ax.add_patch(Ellipse((x, y), 2 * 4 * np.sqrt(w[1]), 2 * 4 * np.sqrt(w[0]), angle=ang, fc='#86b6ef', ec='#1c5cab', lw=1))
        ax.plot([x, 45.4], [y, -54.2], color='#e6e5e0', lw=0.5, zorder=0)
ax.set_xlim(-5, 127); ax.set_ylim(-8, 127); ax.set_aspect('equal'); ax.set_xlabel('x (cm)'); ax.set_ylabel('y (cm)')
ax.set_title('Side camera alone: error shape (drawn 4x)', loc='left'); ax.add_patch(plt.Rectangle((0, 0), W, L, fill=False, ec='#52514e', lw=1))
fig.tight_layout(); fig.savefig(OUT + 'twocam-agreement-map.png', dpi=130); plt.close(fig)
# 4. 09:56 Left
rows = list(csv.DictReader(open('../work/ev_0955left.csv'))); s0 = np.array([77.38, 93.29])
t0 = sess.ep('0955', '09:56:28'); t1 = sess.ep('0955', '09:56:52')
T = []; for_ = []
for r_ in rows:
    t = float(r_['t'])
    if t0 <= t <= t1:
        T.append(t - t0)
        tr = np.array([float(r_['track_x']), float(r_['track_y'])]); sd = np.array([float(r_['side_x']), float(r_['side_y'])]) if r_['side_x'] else np.array([np.nan, np.nan])
        for_.append((np.hypot(*(tr - s0)), np.hypot(*(sd - s0)), float(r_['track_age_s'])))
F = np.array(for_); T = np.array(T)
fig, ax = plt.subplots(figsize=(7.5, 3.8))
ax.plot(T, F[:, 1], color=C[0], lw=2, label='side camera'); ax.plot(T, F[:, 0], color=C[1], lw=2, label='overhead (track.csv)')
st = (F[:, 2] > 0.6) & (T > 10)
ax.fill_between(T, 0, 18, where=st, color='#e6e5e0', step='mid', label='overhead not updating')
ax.axvline(sess.ep('0955', '09:56:41') - t0, color='#52514e', lw=1, ls='--'); ax.text(sess.ep('0955', '09:56:41') - t0 + 0.2, 16.8, 'hold "lost"', fontsize=9, color='#52514e')
ax.text(4.5, 16.8, 'go 12', fontsize=9, color='#52514e'); ax.text(9.6, 16.8, 'go -11.8', fontsize=9, color='#52514e')
ax.set_xlabel('seconds after 09:56:28'); ax.set_ylabel("Left's distance from its start (cm)"); ax.set_ylim(0, 18)
ax.legend(frameon=False, loc='upper right', bbox_to_anchor=(1, 0.9)); ax.set_title('Left, 26 Oct 09:56: out 12 cm and back', loc='left')
fig.tight_layout(); fig.savefig(OUT + 'twocam-0956-left-out-and-back.png', dpi=130); plt.close(fig)
# 6. lag scans
fig, ax = plt.subplots(figsize=(7.5, 3.8))
for (s, c) in zip(['1510', '1545', '2114', '0955'], C):
    sc = pickle.load(open('../work/cal_%s.pkl' % s, 'rb'))['scan']; a = np.array(sc)
    ax.plot(a[:, 0], a[:, 2], color=c, lw=2, label={'1510': '25 Oct 15:10', '1545': '25 Oct 15:45', '2114': '25 Oct 21:14', '0955': '26 Oct 09:55'}[s])
ax.set_xlabel('Shift applied to the overhead positions (s); the minimum is the lag'); ax.set_ylabel('Side px miss, moving robots (median)')
ax.legend(frameon=False); ax.set_title('Time lag between the two cameras, per session', loc='left')
fig.tight_layout(); fig.savefig(OUT + 'twocam-clock-lag.png', dpi=130); plt.close(fig)
# 7. Pi cost
b = json.load(open('../work/bench.json')); rates = np.array([1, 2, 3, 5, 10, 15])
fig, ax = plt.subplots(figsize=(7.5, 3.8))
for key, c, lab in [('decode+blobs_full', C[1], 'whole side picture'), ('cheap_confirm', C[0], 'half-size crop at the expected spot')]:
    lo = rates * b[key] * 1.9 / 1000 * 100; hi = rates * b[key] * 4.8 / 1000 * 100
    ax.fill_between(rates, lo, hi, color=c, alpha=0.2); ax.plot(rates, lo, color=c, lw=2, label=lab)
ax.set_yscale('log'); ax.set_xlabel('Side pictures checked a second'); ax.set_ylabel('% of one Pi core (log)')
ax.set_title('Pi cost of using the side camera (line: idle Pi; band: up to loaded Pi)', loc='left', fontsize=10); ax.legend(frameon=False, loc='upper left')
fig.tight_layout(); fig.savefig(OUT + 'twocam-pi-cost.png', dpi=130); plt.close(fig)
print('ok')
