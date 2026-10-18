"""Make tools/blink and its tests importable; shared synthetic recordings."""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))


@pytest.fixture(scope="session")
def scenes(tmp_path_factory):
    """Render each synthetic scenario once and run blink.py on it.
    Returns {name: (folder, tracks, segs, lights, rows, truth)}."""
    import blink
    import score
    import synthworld
    out = {}
    root = tmp_path_factory.mktemp("blink")
    for name in ("clutter", "crossing", "distractors"):
        d = str(root / name)
        sc, _ = synthworld.scenario(name, seed=0)
        sc.render(d)
        p = blink.Params()
        tracks, segs, lights, ft = blink.analyse(d, p)
        rows = blink.rows_for(tracks, ft, p)
        blink.write_rows(os.path.join(d, "blink_detections.csv"), rows)
        out[name] = dict(folder=d, tracks=tracks, segs=segs, lights=lights, p=p, frame_t=ft,
                         rows=blink.read_blink_csv(os.path.join(d, "blink_detections.csv")),
                         truth=score.read_truth(os.path.join(d, "truth_lights.csv")))
    return out
