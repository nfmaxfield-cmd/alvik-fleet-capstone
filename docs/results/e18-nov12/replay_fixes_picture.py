"""e18b: one terminal-style picture of the four fixes from the coordinator's replay of all 10 home runs (12 Nov), each
on and off, from the test rig and the tracking layer. python3 replay_fixes_picture.py OUT_DIR"""
import os, random, sys, types
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.join(REPO, "tools", "fleet", "tests")); sys.path.insert(0, os.path.join(REPO, "tools", "fleet"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import freeroam_core as FC
import odo_trust as OT
import test_e18_safety_blind as T


class MP:
    """monkeypatch.setattr stand-in."""
    def setattr(self, obj, name, val):
        setattr(obj, name, val)


def pic(lines, path, title):
    fig = plt.figure(figsize=(11, 0.22 * len(lines) + 0.6))
    fig.patch.set_facecolor("#1a1a19")
    fig.text(0.01, 0.98, title, color="#f0efec", family="monospace", fontsize=8.5, va="top", weight="bold")
    for k, l in enumerate(lines):
        fig.text(0.01, 0.98 - (k + 1.6) / (len(lines) + 2.2), l[:165], color="#c3c2b7", family="monospace",
                 fontsize=7.3, va="top")
    fig.savefig(path, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)


mp = MP()
L = ["== 1. 5 Nov 14:55: Left+Center in a close pass, all three unaccounted, a spot on Left's OWN pose; Right 56 cm away =="]
old, _ = T._elim_scene(False)
new, ref = T._elim_scene(True)
L.append("   MIRLT_ELIM_PAIR_GUARD=0 (e17): named %s" % (sorted(set(old)) or "nobody"))
L.append("   MIRLT_ELIM_PAIR_GUARD=1 (e18): named %s; the elimination refused %d times" % (sorted(set(new)) or "nobody", ref))
L.append("")
L.append("== 2. 11 Nov 08:26: Center standing, its own pose 31 cm off, the layer naming it all along ==")
for on in (True, False):
    R, ev = T._far_rig(on, mp)
    first = [e for e in R.events if e[1] == "Center" and e[2] in ("CORRECTED", "lost", "seen again", "REJOINED")][:3]
    L.append("   MIRLT_FAR_NAMED_CORRECT=%d:" % on)
    L += ["     [%5.1f s] %s: %s" % (e[0], e[2], e[3][:120]) for e in first]
FC.FAR_NAMED_CORRECT = True
L.append("")
L.append("== 3. a re-seed of a standing robot (own heading 90 deg, the camera says 130) ==")
for lab, kw in (("roof tag, heading fresh", dict()), ("lights outline", dict(src="lights")),
                ("roof tag, ten quarter turns since", dict(turned=900.0))):
    L.append("   MIRLT_RESEED_HEADING_KEEP=1, %-34s -> fpose heading %.0f deg" % (lab, T._reseed(True, mp, **kw)))
L.append("   MIRLT_RESEED_HEADING_KEEP=0, %-34s -> fpose heading %.0f deg" % ("roof tag, heading fresh", T._reseed(False, mp)))
FC.RESEED_HEADING_KEEP = True
L.append("")
L.append("== 4. the trust record: Left and Center drift 0.06 cm/cm, Right 0.30 (30 samples each) ==")
rng = random.Random(3)
fl = OT.link({n: OT.Trust(n) for n in ("Left", "Center", "Right")})
for k in range(30):
    s = (10.0, 20.0, 40.0)[k % 3]
    fl["Left"].add_sample(s, (2.3 + 0.06 * s) * rng.uniform(0.5, 1.2))
    fl["Center"].add_sample(s, (2.3 + 0.06 * s) * rng.uniform(0.5, 1.2))
    fl["Right"].add_sample(s, (2.0 + 0.30 * s) * rng.uniform(0.5, 1.2))
for n in fl:
    L.append("   " + fl[n].words())
pic(L, os.path.join(sys.argv[1], "e18-feature-replay-fixes-on-and-off-terminal.png"),
    "Simulated (test rig and tracking layer): the four fixes from the replay of all 10 home runs, each on and off")
print("\n".join(L))
