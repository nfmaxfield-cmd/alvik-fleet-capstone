"""pose.py : camera positions in the stored calibration's board cm (roam1: x = 0 the bottom open edge, y = 0 the right
one), as the border used them. Explore's positions (before 22:02:12) are on the provisional camera: back to pixels,
then onto roam1 at the robot's top height, as e02.border_frame does. After explore failed the runner itself used
roam1, so those are taken as they are."""
import os, sys, csv, bisect
HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, EXP)
sys.path.insert(0, os.path.join(os.path.dirname(EXP), 'recognition'))
import numpy as np
import e02_camera as CAM
import e02_explore as EX
from common import D, ep, lt
EXPLORE_END = ep('22:02:12.5')
PROV = None
ROAM = CAM.load_camera(os.path.join(EXP, 'data', 'overhead_floorcal_roam1.json'))


def prov():
    global PROV
    if PROV is None:
        import importlib.util
        import cameras as CMS, math
        f = 600.0
        PROV = CMS.CameraModel([[f, 0, 320.0], [0, f, 240.0], [0, 0, 1]], [0, 0, 0, 0, 0], [math.pi, 0.0, 0.0],
                               [0.0, 0.0, 260.0], (640, 480), name="provisional")
    return PROV


def to_board(t, x, y):
    """(x, y) cm on the board (roam1) and the pixel (u, v) of the robot's top."""
    if t < EXPLORE_END:
        u, v = prov().project((x, y, EX.ROBOT_TOP_CM))
    else:
        u, v = ROAM.project((x, y, EX.ROBOT_TOP_CM))
    gx, gy = ROAM.to_floor(float(u), float(v), EX.ROBOT_TOP_CM)
    return float(gx), float(gy), float(u), float(v)


def board_px(x, y, z=0.0):
    u, v = ROAM.project((x, y, z))
    return float(u), float(v)


def track(robot=None, t0=None, t1=None):
    out = []
    with open(os.path.join(D, 'track.csv')) as f:
        for r in csv.DictReader(f):
            t = float(r['t'])
            if (t0 and t < t0) or (t1 and t > t1) or (robot and r['robot'] != robot):
                continue
            try:
                x, y = float(r['x_cm']), float(r['y_cm'])
            except ValueError:
                continue
            gx, gy, u, v = to_board(t, x, y)
            out.append(dict(t=t, robot=r['robot'], x=x, y=y, bx=gx, by=gy, u=u, v=v, src=r['source'],
                            phase=r['phase']))
    return out


if __name__ == '__main__':
    for q in track(sys.argv[1], ep(sys.argv[2]), ep(sys.argv[3]))[::int(sys.argv[4]) if len(sys.argv) > 4 else 1]:
        print(lt(q['t']), q['robot'], '%.1f %.1f -> board %.1f %.1f px %.0f %.0f %s' % (
            q['x'], q['y'], q['bx'], q['by'], q['u'], q['v'], q['src']))
