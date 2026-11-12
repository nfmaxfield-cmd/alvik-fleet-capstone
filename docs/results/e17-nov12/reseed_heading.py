"""For each fpose the run sent (pose.csv reseed rows; events heading re-set / off plan heading re-set), the robot's own
heading discontinuity when the fpose landed (follow.csv), against how fast it was turning when it was sent."""
import csv, sys, math, bisect, json
R = sys.argv[1]
w = lambda a: (a + 180) % 360 - 180
fo = {}
for r in csv.DictReader(open(R + '/follow.csv')):
    if r['state'] == 'nopose': continue
    fo.setdefault(r['robot'], []).append((float(r['t_recv']), float(r['x_cm']), float(r['y_cm']), float(r['th_deg']), r['state']))
sends = []
for r in csv.DictReader(open(R + '/pose.csv')):
    if r['what'] == 'reseed': sends.append((float(r['t']), r['robot'], 'reseed'))
for r in csv.DictReader(open(R + '/events.csv')):
    if r['kind'] in ('heading re-set',) or (r['kind'] == 'off plan' and 'heading re-set' in r['text']):
        sends.append((float(r['t']), r['robot'], 'heading'))
out = []
for t, n, kind in sorted(sends):
    F = fo.get(n, []); ts = [q[0] for q in F]
    i = bisect.bisect(ts, t) - 1
    if i < 3 or i + 15 >= len(F): continue
    # turn rate at send: last 0.5 s of own records before t
    a = max(0, i - 5); rate = w(F[i][3] - F[a][3]) / max(0.05, F[i][0] - F[a][0])
    # find the landing: the first step after t with a position jump > 2.5 cm in one record
    land = None
    for k in range(i + 1, min(len(F) - 1, i + 15)):
        if math.hypot(F[k][1] - F[k-1][1], F[k][2] - F[k-1][2]) > 2.5 and F[k][0] - F[k-1][0] < 0.4:
            land = k; break
    if land is None: continue
    k = land
    # heading discontinuity: the step's change minus the expected change at the local rate (records around it)
    r0 = w(F[k-1][3] - F[k-2][3]) / max(0.05, F[k-1][0] - F[k-2][0]); r1 = w(F[k+2][3] - F[k+1][3]) / max(0.05, F[k+2][0] - F[k+1][0])
    exp = 0.5 * (r0 + r1) * (F[k][0] - F[k-1][0])
    disc = w(F[k][3] - F[k-1][3] - exp)
    out.append(dict(t=t, robot=n, kind=kind, rate=rate, state=F[i][4], lag=F[k][0] - t, disc=disc))
mv = [o for o in out if abs(o['rate']) > 20]; st = [o for o in out if abs(o['rate']) <= 5]
med = lambda v: sorted(v)[len(v)//2] if v else float('nan')
print(json.dumps(dict(n=len(out), turning=len(mv), standing=len(st),
    turning_abs_disc_median=round(med([abs(o['disc']) for o in mv]),1), standing_abs_disc_median=round(med([abs(o['disc']) for o in st]),1),
    turning_over15=sum(abs(o['disc'])>15 for o in mv), standing_over15=sum(abs(o['disc'])>15 for o in st),
    corr_rate_disc=round(sum(o['rate']*o['disc'] for o in mv)/max(1e-9,(sum(o['rate']**2 for o in mv)*sum(o['disc']**2 for o in mv))**.5),2) if mv else None)))
with open(sys.argv[2], 'w') as f: json.dump(out, f)
