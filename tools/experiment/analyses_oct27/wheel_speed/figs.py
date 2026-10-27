"""figs.py OUTDIR : the report's four figures (PNG) from OUT/stretches.csv, OUT/legs.csv and the regressions.
 1 stretch_error_vs_speed.png : the trust check's stretches, signed and absolute wheel error against wheel speed, per
                                robot, with each robot's fitted line (covariates held at their means).
 2 leg_ratio_by_commanded.png : every go leg, camera distance over wheel distance by commanded speed and robot.
 3 residuals.png              : residuals of the absolute-error model against fitted values and against speed.
 4 session_spread.png         : each robot's stretch error spread (robust sd) per session, against the healthy 0.6 cm."""
import sys, os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import regress as RG
from common import short

OUTD = sys.argv[1]
os.makedirs(OUTD, exist_ok=True)
COL = {'Left': '#2a78d6', 'Center': '#eb6834', 'Right': '#1baf7a'}
MK = {'Left': 'o', 'Center': 's', 'Right': '^'}
INK, INK2, SURF, GRID = '#0b0b0b', '#52514e', '#fcfcfb', '#e6e5e0'
plt.rcParams.update({'font.size': 10, 'axes.edgecolor': '#b5b4ad', 'axes.labelcolor': INK2, 'xtick.color': INK2,
                     'ytick.color': INK2, 'figure.facecolor': SURF, 'axes.facecolor': SURF})
A, B = RG.prep_a(), RG.prep_b()
COV = ['length', 'kind', 'session', 'battery']


def style(ax):
    ax.grid(color=GRID, lw=0.8); ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)


def src(fig, text):
    fig.text(0.01, 0.01, text, fontsize=7.5, color=INK2, ha='left', va='bottom')


# 1 -------------------------------------------------------------------------------------------------------------
fig, axs = plt.subplots(1, 2, figsize=(10.5, 4.6))
for ax, resp, lab in ((axs[0], 'err_cm', 'Wheel error, signed (cm)\ncamera length - wheels x stored scale'),
                      (axs[1], 'abs_err', 'Wheel error, size (cm)')):
    for r in RG.ROBOTS:
        d = A[A.robot == r]
        X, nm = RG.design(d, 'wheel_speed', COV)
        f = RG.ols(d[resp].values, X, nm)
        xs = np.linspace(d.wheel_speed.quantile(0.02), d.wheel_speed.quantile(0.98), 20)
        Xm = X[f['keep']].mean(0)
        idx = [nm.index(n) for n in f['names']]
        base = Xm[idx] @ f['b']
        sl = f['coef']['wheel_speed']['b']
        ys = base + sl * (xs - d.wheel_speed[f['keep']].mean())
        lo, hi = f['coef']['wheel_speed']['lo'], f['coef']['wheel_speed']['hi']
        ax.scatter(d.wheel_speed, d[resp], s=14, marker=MK[r], color=COL[r], alpha=0.5, linewidths=0,
                   label='%s: %+.2f [%+.2f, %+.2f]' % (r, sl, lo, hi))
        ax.plot(xs, ys, color=COL[r], lw=2)
    ax.axhline(0, color=INK2, lw=0.8)
    ax.set_xlabel('Wheel-measured speed over the stretch (cm/s)')
    ax.set_ylabel(lab)
    ax.set_xlim(3.5, 10)
    style(ax)
axs[0].set_ylim(-8, 8); axs[1].set_ylim(0, 8)
axs[0].set_title('Signed: no trend with speed', loc='left', fontsize=10.5, color=INK)
axs[1].set_title('Size: grows about 0.2 cm per cm/s, all three alike', loc='left', fontsize=10.5, color=INK)
axs[0].legend(frameon=False, loc='lower left', fontsize=8.5, title='slope, 95 % CI', title_fontsize=8.5)
axs[1].legend(frameon=False, loc='upper left', fontsize=8.5, title='slope, 95 % CI', title_fontsize=8.5)
fig.suptitle("The trust check's stretches: wheel error against speed (lines: each robot's fit, slope in cm per cm/s)",
             x=0.01, ha='left', fontsize=11, color=INK)
src(fig, 'Measured: camera straight-stretch length vs the wheels\' raw count x the session\'s stored wheel scale; 338 '
         'stretches in find and route runs.\nData: track.csv + robot-logs-1/raw.csv, sessions Oct 24 15:36, 16:16, '
         'Oct 25 15:45, 21:14 (GitHub releases).')
fig.tight_layout(rect=(0, 0.07, 1, 0.95))
fig.savefig(os.path.join(OUTD, 'stretch_error_vs_speed.png'), dpi=130); plt.close(fig)

# 2 -------------------------------------------------------------------------------------------------------------
L = pd.read_csv(os.path.join(RG.OUT, 'legs.csv'))
L = L[~L.fix & L.cms.notna() & (L.wheel_cm >= 1.0)].copy()
L['cam_use'] = np.where(L.still, L.cam_cm, np.where(L.cms <= 4, L.cam_at_cm, np.nan))
L = L[L.cam_use.notna()]
L['ratio'] = (L.cam_use / L.wheel_cm).clip(upper=2.0)
fig, ax = plt.subplots(figsize=(9.8, 4.8))
rng = np.random.default_rng(3)
levels = [(2.0, 'creep steps, 2 cm/s\n(3 cm pushes towards a board)'),
          (4.0, 'crawl, 4 cm/s\n(rays, wiggles, squares, back-outs)'),
          (10.0, 'normal, 10 cm/s\n(returns, parking, route legs)')]
for i, (lv, _) in enumerate(levels):
    for j, r in enumerate(RG.ROBOTS):
        d = L[(L.robot == r) & (L.cms == lv)]
        x0 = i * 4 + j
        ax.scatter(x0 + rng.uniform(-0.28, 0.28, len(d)), d.ratio, s=13, marker=MK[r], color=COL[r], alpha=0.55,
                   linewidths=0, label=r if i == 0 else None)
        if len(d):
            med = float(d.ratio.median())
            ax.plot([x0 - 0.35, x0 + 0.35], [med, med], color=INK, lw=2)
        nst = int((d.ratio < 0.5).sum())
        ax.text(x0, -0.13, '%d/%d' % (nst, len(d)), fontsize=8, ha='center', color=INK2)
ax.axhline(1.0, color=INK2, lw=0.8)
ax.axhline(0.5, color=INK2, lw=0.8, ls='--')
ax.text(11.6, 0.52, 'body moved under half\nof what the wheels counted', fontsize=8, color=INK2, ha='right', va='bottom')
ax.text(-0.6, -0.13, 'below line:', fontsize=8, color=INK2, ha='right')
ax.set_xticks([1, 5, 9]); ax.set_xticklabels([t for _, t in levels])
ax.set_ylim(-0.2, 2.05); ax.set_xlim(-1.6, 11.7)
ax.set_ylabel('Camera distance / wheel distance, per leg')
style(ax)
ax.legend(frameon=False, loc='upper left', bbox_to_anchor=(0.2, 1.0), fontsize=9, ncol=3, markerscale=1.6)
ax.set_title('Every go leg by commanded speed: stalls are creep steps, mostly Left\'s (black bar: median)', loc='left',
             fontsize=11, color=INK)
src(fig, 'Measured: camera position after a leg minus before it (standing; slow legs without standing pictures: at the '
         'leg\'s start and end times) over the robot\'s own pose change.\nData: leg.csv, track.csv, e02_log.txt '
         '(commanded speed), 8 sessions Oct 24-26 (GitHub releases). Ratios above 2 drawn at 2.')
fig.tight_layout(rect=(0, 0.07, 1, 1))
fig.savefig(os.path.join(OUTD, 'leg_ratio_by_commanded.png'), dpi=130); plt.close(fig)
print(L.assign(st=L.ratio < 0.5).groupby(['cms', 'robot']).st.agg(['sum', 'size']).unstack())

# 3 -------------------------------------------------------------------------------------------------------------
fig, axs = plt.subplots(1, 2, figsize=(10.5, 4.3))
for r in RG.ROBOTS:
    d = A[A.robot == r]
    X, nm = RG.design(d, 'wheel_speed', COV)
    f = RG.ols(d.abs_err.values, X, nm)
    axs[0].scatter(f['fitted'], f['resid'], s=13, marker=MK[r], color=COL[r], alpha=0.55, linewidths=0, label=r)
    axs[1].scatter(d.wheel_speed[f['keep']], f['resid'], s=13, marker=MK[r], color=COL[r], alpha=0.55, linewidths=0)
for ax, xl in ((axs[0], 'Fitted wheel error size (cm)'), (axs[1], 'Wheel-measured speed (cm/s)')):
    ax.axhline(0, color=INK2, lw=0.8); ax.set_xlabel(xl); ax.set_ylabel('Residual (cm)'); style(ax)
axs[0].legend(frameon=False, fontsize=9)
axs[0].set_title('Residuals fan out as the fit rises', loc='left', fontsize=10.5, color=INK)
axs[1].set_title('No curve left over against speed', loc='left', fontsize=10.5, color=INK)
src(fig, 'Measured: what the per-robot model (error size ~ speed + length + kind + session + battery) leaves unexplained, '
         'per stretch. Data: as figure 1 (338 stretches).')
fig.tight_layout(rect=(0, 0.07, 1, 1))
fig.savefig(os.path.join(OUTD, 'residuals.png'), dpi=130); plt.close(fig)

# 4 -------------------------------------------------------------------------------------------------------------
g = A.groupby(['session', 'robot']).agg(n=('err_cm', 'size'), rsd=('err_cm', lambda v: np.median(np.abs(v)) / 0.6745),
                                         spd=('wheel_speed', 'median')).reset_index()
g = g[g.n >= 5]
sess = sorted(g.session.unique())
fig, ax = plt.subplots(figsize=(8.6, 4.3))
for j, r in enumerate(RG.ROBOTS):
    d = g[g.robot == r]
    xs = [sess.index(s) + (j - 1) * 0.18 for s in d.session]
    ax.scatter(xs, d.rsd, s=60, marker=MK[r], color=COL[r], label=r, zorder=3)
    for x, y, n in zip(xs, d.rsd, d.n):
        ax.text(x, y + 0.07, '%.2f' % y, ha='center', fontsize=8, color=INK)
ax.axhline(0.6, color=INK2, ls='--', lw=1)
ax.text(len(sess) - 0.55, 0.62, 'healthy wheels alone: 0.6 cm', fontsize=8.5, color=INK2, ha='right', va='bottom')
ax.set_xticks(range(len(sess))); ax.set_xticklabels([short(s) for s in sess])
ax.set_ylim(0, 2.2); ax.set_ylabel('Stretch wheel error spread (cm, robust sd)')
ax.set_xlim(-0.6, len(sess) - 0.4)
style(ax); ax.legend(frameon=False, fontsize=9, loc='upper left')
ax.set_title('Spread per robot and session: Left is the noisiest, not Center', loc='left', fontsize=11, color=INK)
src(fig, 'Measured: median |camera - wheels x stored scale| / 0.6745 per robot and session (5+ stretches). Data: as '
         'figure 1.\n0.6 cm is e02_trust.HEALTHY["wheels"], the wheels\' own share without the camera\'s.')
fig.tight_layout(rect=(0, 0.07, 1, 1))
fig.savefig(os.path.join(OUTD, 'session_spread.png'), dpi=130); plt.close(fig)
print(g.to_string())
print('written', OUTD)
