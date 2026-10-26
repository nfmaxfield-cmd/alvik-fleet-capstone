"""avindex.py N : build idx/frames_idx.txt (absolute byte offset and size of every frame) from the first N OpenDML
ix00 chunks of the cam0 video, fetched by byte range (no whole-part download)."""
import struct, subprocess, sys, os
from common import W
HERE = os.path.dirname(os.path.abspath(__file__))
os.makedirs(W + '/idx', exist_ok=True)
d = open(W + '/vid/head.bin', 'rb').read()
o = d.find(b'indx')
nEnt = struct.unpack('<I', d[o + 12:o + 16])[0]
sup = [struct.unpack('<QII', d[o + 32 + 16 * i:o + 48 + 16 * i]) for i in range(nEnt)]
out = open(W + '/idx/frames_idx.txt', 'w')
n = 0
for off, size, dur in sup[:int(sys.argv[1])]:
    tmp = W + '/idx/_ix.bin'
    subprocess.run([HERE + '/rng.sh', str(off), str(size), tmp], check=True)
    b = open(tmp, 'rb').read()
    assert b[:4] == b'ix00', b[:4]
    wl, bs, bt, ne = struct.unpack('<HBBI', b[8:16])
    base = struct.unpack('<Q', b[20:28])[0]
    for i in range(ne):
        so, sz = struct.unpack('<II', b[32 + 8 * i:40 + 8 * i])
        out.write('%d %d\n' % (base + so, sz & 0x7fffffff))
        n += 1
print('frames indexed', n)
