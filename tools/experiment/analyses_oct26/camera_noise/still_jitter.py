"""still_jitter.py <track.csv> [t_from t_to] : the camera's wobble on a robot standing still, in the border check's
own terms (2026-10-26 runner fixes, item 2). The border check (e02.py border_check) looks at the last 0.6 s of tracker
positions and asks how far the robot came towards an open edge. Here: for every 0.6 s window while the followed robot
stands still, the move along x and along y between the window's first and last position (single pictures, as the
check used them) and between the medians of its first 3 and last 3 positions, and the move of a least-squares straight
line through all of them (what the check uses since 2026-10-26). Percentiles of the larger of |dx|, |dy|.
Default window: 2026-10-25 21:14 session's overnight hold, 23:00-03:00 (Right still, followed, 10 pictures a second)."""
import csv, sys, statistics as S
f = sys.argv[1]
a = float(sys.argv[2]) if len(sys.argv) > 2 else 1792983600.0     # 2026-10-25 23:00 EDT
b = float(sys.argv[3]) if len(sys.argv) > 3 else 1792998000.0     # 2026-10-26 03:00 EDT
rows = []
for r in csv.DictReader(open(f)):
    try:
        t = float(r['t'])
    except (KeyError, ValueError):
        continue
    if a <= t <= b:
        rows.append((t, float(r['x_cm']), float(r['y_cm'])))
rows.sort()
single, med, fit = [], [], []


def fitted_move(w, k):
    """End minus start of the least-squares straight line through the window's positions (axis k)."""
    ts = [q[0] for q in w]; vs = [q[k] for q in w]
    tm = sum(ts) / len(ts); vm = sum(vs) / len(vs)
    sxx = sum((t - tm) ** 2 for t in ts)
    if sxx <= 0:
        return 0.0
    b = sum((t - tm) * (v - vm) for t, v in zip(ts, vs)) / sxx
    return b * (ts[-1] - ts[0])

i = 0
for j in range(len(rows)):
    while rows[j][0] - rows[i][0] > 0.6:
        i += 1
    w = rows[i:j + 1]
    if len(w) < 6 or rows[j][0] - rows[i][0] < 0.5:
        continue
    single.append(max(abs(w[-1][1] - w[0][1]), abs(w[-1][2] - w[0][2])))
    m0 = (S.median(q[1] for q in w[:3]), S.median(q[2] for q in w[:3]))
    m1 = (S.median(q[1] for q in w[-3:]), S.median(q[2] for q in w[-3:]))
    med.append(max(abs(m1[0] - m0[0]), abs(m1[1] - m0[1])))
    fit.append(max(abs(fitted_move(w, 1)), abs(fitted_move(w, 2))))
for name, v in (('single pictures', single), ('medians of 3', med), ('straight-line fit', fit)):
    v.sort()
    pc = lambda p: v[min(len(v) - 1, int(p * (len(v) - 1)))]
    print('%-16s n %6d  p50 %.2f  p99 %.2f  p99.9 %.2f  p99.99 %.2f  max %.2f cm' % (
        name, len(v), pc(.5), pc(.99), pc(.999), pc(.9999), v[-1]))
xs = [r[1] for r in rows]; ys = [r[2] for r in rows]
print('positions %d, sd x %.2f y %.2f cm' % (len(rows), S.pstdev(xs), S.pstdev(ys)))
