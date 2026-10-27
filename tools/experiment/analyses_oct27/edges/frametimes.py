"""Frame times of each camera from the recorder's frames.csv (shared Pi clock) and the frame nearest a wall time."""
import bisect, math
from common import REC
_ft = {}


def ftimes(cam):
    """list of t_wall per frame index (gaps in frames.csv filled by interpolation)"""
    if cam not in _ft:
        tw = []
        for line in open('%s/%s/frames.csv' % (REC, cam), 'rb').read().split(b'\n')[1:]:
            p = line.replace(b'\x00', b'').strip().split(b',')
            try:
                k, t = int(p[0]), float(p[1])
            except (ValueError, IndexError):
                continue
            while len(tw) < k:
                tw.append(float('nan'))
            if len(tw) == k:
                tw.append(t)
        for i, t in enumerate(tw):
            if math.isnan(t):
                j = i
                while math.isnan(tw[j]):
                    j += 1
                for q in range(i, j):
                    tw[q] = tw[i - 1] + (tw[j] - tw[i - 1]) * (q - i + 1) / (j - i + 1)
        _ft[cam] = tw
    return _ft[cam]


def frame_at(cam, t):
    tw = ftimes(cam)
    i = bisect.bisect_left(tw, t)
    c = [j for j in (i - 1, i) if 0 <= j < len(tw)]
    return min(c, key=lambda j: abs(tw[j] - t))
