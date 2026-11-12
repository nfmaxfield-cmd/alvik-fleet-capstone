# The audit's fuser-alone check (work/hf.py), plus a correlated case: a standing robot whose detector error is the SAME
# in every picture (a bias drawn once per trial from the detector's own error, sd 13.3 deg = 95 % 26 deg) plus 5 deg of
# picture-to-picture noise. Own heading correct throughout: every hand-on is a WRONG one.
import sys, random
sys.path.insert(0, sys.argv[1] + "/tools/fleet")
import tagfree as TF
N = int(sys.argv[2]) if len(sys.argv) > 2 else 1000
def run(sd, hz, secs, bias_sd=0.0, pic_sd=None):
    handed = checks = 0; sizes = []
    for s in range(N):
        rng = random.Random(s); f = TF.HeadingFuser(); t = 0.0; own = 40.0
        b = rng.gauss(0, bias_sd) if bias_sd else 0.0
        while t < secs:
            t += 1.0 / hz
            th, sig, verdict = f.update("A", t, own, own + b + rng.gauss(0, pic_sd if pic_sd is not None else sd))
            adv = f.advice("A")
            if adv == "hand":
                handed += 1; sizes.append(abs(TF.wrap180(th - own))); break
            if adv == "check" or f.take_new_doubt("A"):
                checks += 1; break
    sizes.sort()
    return 100.0 * handed / N, (sizes[len(sizes) // 2] if sizes else 0), 100.0 * checks / N
for label, kw in (("independent 15 deg", dict(sd=15.0)), ("independent 10 deg", dict(sd=10.0)),
                  ("independent 18 deg", dict(sd=18.0)), ("same error every picture (13.3 deg bias + 5 deg)", dict(sd=0, bias_sd=13.3, pic_sd=5.0))):
    for secs in (15, 60, 300):
        h, m, c = run(kw.get("sd", 0), 2.0, secs, kw.get("bias_sd", 0.0), kw.get("pic_sd"))
        print("%-48s %3d s standing, own heading right: wrong hand-on %5.1f %% (median %2.0f deg), check asked %5.1f %%" % (label, secs, h, m, c))
