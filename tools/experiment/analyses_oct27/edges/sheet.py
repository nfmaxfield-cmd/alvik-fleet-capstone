"""sheet.py OUT.jpg STAGE 'HH:MM:SS[.s],...' [crop x0,y0,x1,y1 px] [cols] : overhead (cam0) frames at those times with the
board's open-edge lines (red, roam1 at the floor) and each robot's track.csv spot at that moment (circle; dashed =
'held'), labelled with the time; tiled into one picture."""
import sys, os, bisect
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..')); sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'recognition'))
import numpy as np, cv2
import e02_camera as CAM
import vid, frametimes as FT
from common import TAG, ep, track, XMAX, YMAX, REPO
ROAM = CAM.load_camera(os.path.join(REPO, 'tools/experiment/data/overhead_floorcal_roam1.json'))
COL = {'Left': (60, 200, 255), 'Center': (80, 255, 80), 'Right': (255, 120, 255)}


def px(x, y, z=0.0):
    u, v = ROAM.project((x, y, z)); return int(round(float(u))), int(round(float(v)))


def board(img):
    pts = [px(x, y) for x, y in ((0, 0), (XMAX, 0), (XMAX, YMAX), (0, YMAX))]
    cv2.polylines(img, [np.array(pts)], True, (0, 0, 255), 1)


def pos(tk, robot, t, maxdt=0.6):
    a = tk.get(robot, []); ts = [r[0] for r in a]
    i = bisect.bisect_left(ts, t)
    c = [j for j in (i - 1, i) if 0 <= j < len(a)]
    if not c: return None
    j = min(c, key=lambda j: abs(ts[j] - t))
    return a[j] if abs(ts[j] - t) <= maxdt else None


def render(stage, times, crop=None, robots=('Left', 'Center', 'Right')):
    tk = track(stage) if stage else {}
    ks = {FT.frame_at('cam0', ep(s)): s for s in times}
    fr = vid.frames(TAG, 'cam0', list(ks))
    tiles = []
    for k, s in sorted(ks.items(), key=lambda kv: kv[1]):
        img = cv2.imdecode(np.frombuffer(fr[k], np.uint8), cv2.IMREAD_COLOR)
        board(img)
        for r in robots:
            p = pos(tk, r, ep(s))
            if p:
                u, v = px(p[1], p[2], CAM.ROBOT_TOP_CM)
                cv2.circle(img, (u, v), 9, COL[r], 1 if p[3] == 'held' else 2)
                cv2.putText(img, r[0], (u + 10, v), cv2.FONT_HERSHEY_SIMPLEX, 0.45, COL[r], 1)
        if crop:
            x0, y0, x1, y1 = crop; img = cv2.resize(img[y0:y1, x0:x1], None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        cv2.putText(img, s, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
        cv2.putText(img, s, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        tiles.append(img)
    return tiles


def tile(tiles, cols):
    h, w = tiles[0].shape[:2]
    rows = [tiles[i:i + cols] for i in range(0, len(tiles), cols)]
    rows[-1] += [np.zeros_like(tiles[0])] * (cols - len(rows[-1]))
    return np.vstack([np.hstack(r) for r in rows])


if __name__ == '__main__':
    out, stage, ts = sys.argv[1], sys.argv[2], sys.argv[3].split(',')
    crop = tuple(map(int, sys.argv[4].split(','))) if len(sys.argv) > 4 and sys.argv[4] != '-' else None
    cols = int(sys.argv[5]) if len(sys.argv) > 5 else 4
    cv2.imwrite(out, tile(render(stage if stage != '-' else None, ts, crop), cols), [cv2.IMWRITE_JPEG_QUALITY, 88])
    print(out)
