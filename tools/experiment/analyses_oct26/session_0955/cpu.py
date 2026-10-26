"""cpu.py : per-minute Pi load (cpu.csv) against the runner's picture rate (perf.csv)."""
import sys, collections, statistics as st
from common import rows, lt, num
cpu = rows('cpu.csv'); perf = rows('perf.csv')
m = collections.defaultdict(lambda: collections.defaultdict(list))
for r in cpu:
    k = lt(r['t'])[:5]
    for c in ('core0', 'core1', 'core2', 'core3', 'temp_c', 'freq_mhz', 'recorder_cpu', 'runner_cpu', 'tracker_cpu', 'load1'):
        v = num(r[c])
        if v is not None:
            m[k][c].append(v)
    m[k]['thr'].append(r['throttled'])
for r in perf:
    k = lt(r['t'])[:5]
    m[k]['fps'].append(float(r['fps_handled'])); m[k]['ms'].append(float(r['ms_mean'])); m[k]['age'].append(float(r['age_mean_s']))
    m[k]['phase'].append(r['phase'])
print('min   fps  ms   age | c0  c1  c2  c3 | rec run trk | temp  MHz  throttled')
for k in sorted(m):
    d = m[k]
    md = lambda c: st.median(d[c]) if d[c] else float('nan')
    thr = collections.Counter(d['thr']).most_common(2)
    print('%s %4.1f %4.0f %.2f | %3.0f %3.0f %3.0f %3.0f | %3.0f %3.0f %3.0f | %4.1f %4.0f %s %s' % (
        k, md('fps'), md('ms'), md('age'), md('core0'), md('core1'), md('core2'), md('core3'), md('recorder_cpu'),
        md('runner_cpu'), md('tracker_cpu'), md('temp_c'), md('freq_mhz'), thr, collections.Counter(d['phase']).most_common(1)))
