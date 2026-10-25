"""getframes.py OUTDIR SPEC : frames by exact index (OpenDML ix00) fetched by byte range. SPEC as avistream."""
import sys, os, subprocess, csv, bisect, calendar, math
HERE = os.path.dirname(os.path.abspath(__file__)); S = os.path.dirname(HERE)
out, spec = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
idx = [tuple(map(int, l.split())) for l in open(os.path.join(S, 'idx/frames_idx.txt'))]
tw = []
for line in open(os.path.join(S, 'dl/rec/20261025-154514_E02/cam0/frames.csv'), 'rb').read().split(b'\n')[1:]:
    p = line.replace(b'\x00', b'').strip().split(b',')
    try: k, t = int(p[0]), float(p[1])
    except (ValueError, IndexError): continue
    while len(tw) < k: tw.append(float('nan'))
    if len(tw) == k: tw.append(t)
for i, t in enumerate(tw):
    if math.isnan(t):
        j = i
        while math.isnan(tw[j]): j += 1
        for q in range(i, j): tw[q] = tw[i-1] + (tw[j] - tw[i-1]) * (q - i + 1) / (j - i + 1)
def ep(s):
    h, m, sec = s.split(':'); return calendar.timegm((2026,10,25,int(h)+4,int(m),0)) + float(sec)
want = []
for w in spec.split(','):
    rng, step = w.split('/'); a, b = rng.split('-')
    want += list(range(bisect.bisect_left(tw, ep(a)), min(bisect.bisect_left(tw, ep(b)), len(idx)), int(step)))
want = sorted(set(i for i in want if not os.path.exists(os.path.join(out, '%06d.jpg' % i))))
# group into byte ranges of at most 40 MB
groups, cur = [], []
for i in want:
    if cur and (idx[i][0] + idx[i][1] - idx[cur[0]][0] > 40e6 or i - cur[-1] > 400):
        groups.append(cur); cur = []
    cur.append(i)
if cur: groups.append(cur)
tmp = os.path.join(out, '_r.bin')
for g in groups:
    s0 = idx[g[0]][0]; e0 = idx[g[-1]][0] + idx[g[-1]][1]
    subprocess.run([os.path.join(HERE, 'rng.sh'), str(s0), str(e0 - s0), tmp], check=True)
    d = open(tmp, 'rb').read()
    for i in g:
        o, n = idx[i]; j = d[o - s0:o - s0 + n]
        if j[:2] != b'\xff\xd8': print('not a jpeg', i)
        open(os.path.join(out, '%06d.jpg' % i), 'wb').write(j)
os.remove(tmp) if os.path.exists(tmp) else None
with open(os.path.join(out, 'times.csv'), 'w') as f:
    for fn in sorted(os.listdir(out)):
        if fn.endswith('.jpg'): k = int(fn[:6]); f.write('%d,%.3f\n' % (k, tw[k]))
print('done', len(want), 'frames in', len(groups), 'requests; last indexed frame time', tw[len(idx)-1])
