"""grab.py CAM OUTDIR HH:MM:SS[.s] [...] : save the frames nearest the given local times as JPEGs (name = time)."""
import sys, os
import vid, frametimes as FT
from common import TAG, ep
cam, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
want = {FT.frame_at(cam, ep(s)): s for s in sys.argv[3:]}
fr = vid.frames(TAG, cam, list(want))
for k, s in want.items():
    open(os.path.join(out, '%s_%s.jpg' % (cam, s.replace(':', ''))), 'wb').write(fr[k])
print('saved', len(fr))
