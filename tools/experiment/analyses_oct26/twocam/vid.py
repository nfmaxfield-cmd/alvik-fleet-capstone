"""Frames of a session's split AVI (cam0/cam1) by byte range via the OpenDML index; cached index per video."""
import struct, os, json, subprocess, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from remote import Remote, BASE, get
W = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'idx')
os.makedirs(W, exist_ok=True)
_rel = {}
_ic = {}
def assets(tag):
    if tag not in _rel:
        f = os.path.join(W, tag + '.json')
        if not os.path.exists(f):
            open(f, 'w').write(subprocess.run(['gh', 'api', 'repos/nfmaxfield-cmd/alvik-fleet-capstone/releases/tags/' + tag], capture_output=True, text=True).stdout)
        _rel[tag] = {a['name']: a['size'] for a in json.load(open(f))['assets']}
    return _rel[tag]
def remote(tag, cam):
    a = assets(tag)
    parts = sorted(n for n in a if n.startswith(cam + '_video.avi.part-'))
    return Remote([BASE + tag + '/' + p for p in parts], [a[p] for p in parts])
def index(tag, cam):
    f = os.path.join(W, '%s_%s.idx' % (tag, cam))
    if (tag, cam) in _ic: return _ic[(tag, cam)]
    if os.path.exists(f):
        _ic[(tag, cam)] = [tuple(map(int, l.split())) for l in open(f)]
        return _ic[(tag, cam)]
    r = remote(tag, cam)
    d = r.raw(0, 1 << 20)
    o = d.find(b'indx')
    if o < 0:
        return _idx1(r, d, f)
    nEnt = struct.unpack('<I', d[o + 12:o + 16])[0]
    sup = [struct.unpack('<QII', d[o + 32 + 16 * i:o + 48 + 16 * i]) for i in range(nEnt)]
    out = []
    for off, size, dur in sup:
        if off + size > r.n:
            break
        b = r.raw(off, size)
        assert b[:4] == b'ix00', b[:4]
        ne = struct.unpack('<I', b[12:16])[0]
        base = struct.unpack('<Q', b[20:28])[0]
        for i in range(ne):
            so, sz = struct.unpack('<II', b[32 + 8 * i:40 + 8 * i])
            if base + so + (sz & 0x7fffffff) <= r.n:
                out.append((base + so, sz & 0x7fffffff))
    open(f, 'w').write(''.join('%d %d\n' % x for x in out))
    return out
def frames(tag, cam, want, gap=8e6):
    """dict frame index -> jpeg bytes; neighbouring frames fetched in one request when close"""
    idx = index(tag, cam); r = remote(tag, cam)
    want = sorted(set(i for i in want if 0 <= i < len(idx)))
    res = {}; groups = []; cur = []
    for i in want:
        if cur and idx[i][0] + idx[i][1] - idx[cur[0]][0] > 30e6 or (cur and idx[i][0] - (idx[cur[-1]][0] + idx[cur[-1]][1]) > gap):
            groups.append(cur); cur = []
        cur.append(i)
    if cur: groups.append(cur)
    for g in groups:
        s0 = idx[g[0]][0]; e0 = idx[g[-1]][0] + idx[g[-1]][1]
        d = r.raw(s0, e0 - s0)
        for i in g:
            o, n = idx[i]; res[i] = d[o - s0:o - s0 + n]
    return res

def _idx1(r, d, f):
    riff = struct.unpack('<I', d[4:8])[0]
    mv = d.find(b'movi') 
    p = 12
    e = r.raw(p, 8)
    while e[:4] != b'idx1':
        sz0 = struct.unpack('<I', e[4:8])[0]; p += 8 + sz0 + (sz0 & 1); e = r.raw(p, 8)
    n = struct.unpack('<I', e[4:8])[0]
    b = r.raw(p + 8, n)
    out = []
    first = None
    for i in range(0, n, 16):
        ck, fl, off, sz = struct.unpack('<4sIII', b[i:i + 16])
        if not ck.endswith(b'dc') and not ck.endswith(b'db'):
            continue
        if first is None:
            first = off
            base = mv if off < mv else -8  # relative to movi fourcc, else absolute
        out.append((off + base + 8, sz))
    open(f, 'w').write(''.join('%d %d\n' % x for x in out))
    return out
