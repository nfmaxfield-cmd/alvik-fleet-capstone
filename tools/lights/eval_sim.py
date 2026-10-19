#!/usr/bin/env python3
"""The table in tools/blink/README.md ("Light patterns"): simulated scenes (sim_scene.py) per pattern, scale
and seed; light pattern vs blink code on the same frames. About 3 minutes.

    python tools/lights/eval_sim.py [--seeds 3]
"""
import argparse
import sys

import numpy as np

import patterns as PT
import sim_scene as S

CASES = [
    ("printed L", "l3", {}, "passing", 3.4),
    ("printed L", "l3", {}, "close", 3.4),
    ("printed L", "l3", {}, "three", 3.4),
    ("printed L", "l3", {}, "passing", 2.6),
    ("printed L", "l3", {}, "passing", 5.0),
    ("wide L", "l3", PT.WIDE_L3, "three", 3.4),
    ("wide L", "l3", PT.WIDE_L3, "passing", 2.6),
    ("strip", "strip", {}, "passing", 3.4),
    ("strip", "strip", {}, "close", 8.0),
]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, default=3)
    args = ap.parse_args(argv)
    print("| pattern | scene | px/cm | named after, s (median / max) | blink code named after, s | wrong name | "
          "> 3 cm off | heading err p95 / max, deg | frames with a named row |")
    print("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for label, mode, fam, scene, scale in CASES:
        fl, fb, wrong, far, herr, share = [], [], 0, 0, [], []
        for seed in range(args.seeds):
            kw = dict(mode=mode, scale=scale, seed=seed)
            if fam:
                kw["family"] = dict(fam)
            r = S.run(S.SCENES[scene](**kw), with_blink=(scale == 3.4))
            wrong += r["wrong"]
            far += r["far"]
            herr += r["herr"]
            for pr in r["per"].values():
                fl.append(np.inf if pr["first_lights"] is None else pr["first_lights"])
                if scale == 3.4:
                    fb.append(np.inf if pr["first_blink"] is None else pr["first_blink"])
                share.append(pr["named"] / max(pr["frames"], 1))

        def med(v):
            if not v:
                return "-"
            v = np.array(v)
            nn = int(np.isinf(v).sum())
            f = v[np.isfinite(v)]
            s = ("%.2f / %.2f" % (np.median(f), f.max())) if len(f) else "never"
            return s + (" (%d never)" % nn if nn and len(f) else "")
        h = np.array(herr)
        print("| %s | %s | %.1f | %s | %s | %d | %d | %s | %.0f%% |" % (
            label, scene, scale, med(fl), med(fb), wrong, far,
            ("%.1f / %.1f" % (np.percentile(h, 95), h.max())) if len(h) else "-", 100 * np.mean(share)))
        sys.stdout.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main())
