"""Investigate one spot of the picture across a whole recording.

Used by the labelling page's Investigate (key Q): "is this an object, or part of the floor?"
It reads a few hundred frames spread over the recording and, for a small circle around the
spot, measures how different it looks from how it usually looks (the median of all the
samples: the spot on the background picture). From that it gives

  - a strip of small pictures of the spot over time (about 40),
  - a line of how much the spot changed over time,
  - the biggest changes as short clips (a few frames each),
  - the spot on the background picture,
  - a verdict in plain words: part of the background / something that moves (and where it
    went) / something put down or passing over (with the moments to jump to).

Nothing here knows what the objects are; it works for any object type.

    res = scan(video, n_frames, times, x, y, r, frame_now, guesses=None, progress=None)
"""
import base64
import math

import numpy as np
import cv2

MAX_SAMPLES = 480        # frames read over the whole recording
N_THUMBS = 40
N_CLIPS = 5
CLIP_FRAMES = 8
SEEK_GAP = 12            # read on (instead of jumping) when the next wanted frame is this close


def _jpeg(img, quality=82):
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode("ascii") if ok else ""


def read_frames(video, frames, on_frame, stop=None):
    """Decode the given frame numbers in order, calling on_frame(frame, image). Jumps ahead when
    the next frame is far away, reads on when it is near (jumping is slower for short gaps)."""
    wanted = sorted(set(int(f) for f in frames))
    cap = cv2.VideoCapture(video)
    pos = 0
    try:
        for f in wanted:
            if stop is not None and stop():
                return
            if f < pos or f - pos > SEEK_GAP:
                cap.set(cv2.CAP_PROP_POS_FRAMES, f)
                pos = f
            while pos < f:
                if not cap.grab():
                    return
                pos += 1
            ok, img = cap.read()
            pos += 1
            if not ok:
                return
            on_frame(f, img)
    finally:
        cap.release()


class _Spot:
    """Crops around the spot: a small patch for measuring, a larger one (with some
    surroundings) for the pictures."""

    def __init__(self, x, y, r, size):
        self.x, self.y, self.r = float(x), float(y), max(3.0, float(r))
        self.W, self.H = size
        self.R = int(math.ceil(self.r))
        self.C = int(math.ceil(max(2.2 * self.r, 24)))         # half-size of the picture crops
        yy, xx = np.mgrid[-self.R:self.R + 1, -self.R:self.R + 1]
        d = np.hypot(xx, yy)
        self.inner = d <= self.r
        ring = (d > self.r) & (d <= self.R + 0.5)
        self.ring = ring if ring.sum() >= 8 else ~self.inner

    def _crop(self, img, h):
        cx, cy = int(round(self.x)), int(round(self.y))
        pad = h + 2
        big = cv2.copyMakeBorder(img, pad, pad, pad, pad, cv2.BORDER_REPLICATE)
        return big[cy + pad - h:cy + pad + h + 1, cx + pad - h:cx + pad + h + 1]

    def patch(self, img):
        g = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return self._crop(g, self.R).astype(np.float32)

    def picture(self, img):
        return self._crop(img, self.C).copy()


def _runs(flags, max_gap=1):
    """Index runs of True, joining runs separated by at most max_gap False samples."""
    runs = []
    start = None
    gap = 0
    for i, v in enumerate(flags):
        if v:
            if start is None:
                start = i
            gap = 0
            end = i
        elif start is not None:
            gap += 1
            if gap > max_gap:
                runs.append((start, end))
                start = None
    if start is not None:
        runs.append((start, end))
    return runs


def _thumb(pic, out=72):
    return cv2.resize(pic, (out, out), interpolation=cv2.INTER_AREA if pic.shape[0] > out else cv2.INTER_LINEAR)


def _track_moves(guesses, frames_window, x, y, r, f_from, f_to, times):
    """Objects the tracker followed onto or off the spot between f_from and f_to (frames).
    Returns a list of dict(obj, cls, came_from=(frame, x, y) or None, went_to=(frame, x, y) or None)."""
    if not guesses:
        return []
    near, pos = set(), {}
    for f in frames_window:
        for g in guesses.get(f, []):
            d = math.hypot(g["x"] - x, g["y"] - y)
            pos.setdefault(g["obj"], []).append((f, g["x"], g["y"], d, g.get("cls")))
            if f_from <= f <= f_to and d <= 1.5 * r:
                near.add(g["obj"])
    out = []
    far = 2.5 * r
    span = max(1, f_to - f_from + 1)
    for obj in sorted(near):
        ps = sorted(pos[obj])
        on = [p for p in ps if f_from <= p[0] <= f_to and p[3] <= 1.5 * r]
        if len(on) < 0.4 * span:
            continue                  # only passed over the spot: not what sits there
        first_on, last_on = on[0][0], on[-1][0]
        before = [p for p in ps if p[0] < first_on and p[3] > far]
        after = [p for p in ps if p[0] > last_on and p[3] > far]
        came = before[-1] if before else None
        went = after[0] if after else None
        if came or went:
            out.append(dict(obj=obj, cls=on[0][4],
                            came_from=None if not came else dict(frame=came[0], x=round(came[1], 1), y=round(came[2], 1),
                                                                 time_s=round(times[came[0]] - times[0], 2)),
                            went_to=None if not went else dict(frame=went[0], x=round(went[1], 1), y=round(went[2], 1),
                                                               time_s=round(times[went[0]] - times[0], 2))))
    return out


def scan(video, n_frames, times, x, y, r, frame_now=None, guesses=None, progress=None, stop=None,
         size=None, max_samples=MAX_SAMPLES):
    """Look at the spot (x, y, radius r, image px) across the recording. progress(done, total)
    is called while frames are read. Returns a dict for the page (pictures as data: URLs)."""
    n_frames = int(n_frames)
    if n_frames <= 0:
        raise ValueError("the recording has no frames")
    if size is None:
        cap = cv2.VideoCapture(video)
        size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 640, int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 480)
        cap.release()
    spot = _Spot(x, y, r, size)
    k = min(n_frames, max_samples)
    samples = sorted(set(np.linspace(0, n_frames - 1, k).round().astype(int).tolist())
                     | ({int(frame_now)} if frame_now is not None and 0 <= int(frame_now) < n_frames else set()))
    thumb_at = set(samples[i] for i in np.linspace(0, len(samples) - 1, min(N_THUMBS, len(samples))).round().astype(int))
    clip_extra = (N_CLIPS * CLIP_FRAMES) if len(samples) > 1 else 0
    total = len(samples) + clip_extra
    patches, thumbs, now_pic = {}, {}, {}
    state = dict(done=0)

    def tick():
        state["done"] += 1
        if progress and (state["done"] % 5 == 0 or state["done"] == total):
            progress(state["done"], total)

    def on_sample(f, img):
        patches[f] = spot.patch(img)
        pic = spot.picture(img)
        if f in thumb_at:
            thumbs[f] = pic
        if frame_now is not None and f == int(frame_now):
            now_pic["pic"] = pic
        tick()
    if progress:
        progress(0, total)
    read_frames(video, samples, on_sample, stop)
    if stop is not None and stop():
        raise InterruptedError("stopped")
    got = [f for f in samples if f in patches]
    if len(got) < 2:
        raise ValueError("could not read the recording's video")

    stack = np.stack([patches[f] for f in got])
    bg = np.median(stack, axis=0)
    bg_ring = float(np.median(bg[spot.ring])) or 1.0
    change = []
    for f in got:
        p = patches[f]
        g = bg_ring / max(1.0, float(np.median(p[spot.ring])))
        g = min(1.6, max(0.6, g))                    # light changes over the whole picture
        change.append(float(np.abs(p * g - bg)[spot.inner].mean()))
    change = np.array(change)
    # noise: what the samples that look like the usual picture differ by (the lower fifth of the
    # values; works whether the spot is usually empty or usually covered)
    low = float(np.percentile(change, 20))
    thr = max(8.0, 2.5 * low + 4.0)
    changed = change > thr
    t0 = times[0] if times else 0.0

    def tt(f):
        return round((times[f] - t0) if times and f < len(times) else f / 30.0, 2)

    # the spot on the background picture: the median of every sample's picture
    bigs = [thumbs[f] for f in sorted(thumbs)]
    bg_pic = np.median(np.stack(bigs), axis=0).astype(np.uint8) if len(bigs) >= 3 else None

    # the biggest changes: where the line jumps most over about a second (or one sample, when
    # the samples are further apart than that)
    spacing = (tt(got[-1]) - tt(got[0])) / max(1, len(got) - 1)
    lag = max(1, int(round(1.0 / spacing))) if spacing > 0 else 1
    lag = min(lag, len(got) - 1)
    jumps = np.abs(change[lag:] - change[:-lag]) if len(got) > lag else np.zeros(0)
    order = list(np.argsort(-jumps)) if len(jumps) else []
    picked = []
    for i in order:
        if jumps[i] <= max(thr * 0.5, 4.0) or len(picked) >= N_CLIPS:
            break
        if all(abs(int(i) - j) > 2 * lag for j in picked):
            picked.append(int(i))
    clips_frames = {}
    for i in picked:
        fa, fb = got[i], got[i + lag]
        clips_frames[i] = sorted(set(np.linspace(fa, fb, CLIP_FRAMES).round().astype(int).tolist()))
    want = sorted(set(f for fs in clips_frames.values() for f in fs))
    clip_imgs = {}

    def on_clip(f, img):
        clip_imgs[f] = spot.picture(img)
        tick()
    read_frames(video, want, on_clip, stop)
    state["done"] = total - 1
    tick()

    clips = []
    for i in sorted(picked, key=lambda i: got[i]):
        fs = [f for f in clips_frames[i] if f in clip_imgs]
        if not fs:
            continue
        kind = "appears" if change[i + lag] > change[i] else "leaves"
        clips.append(dict(frame_from=got[i], frame_to=got[i + lag], time_s=tt(got[i]), time_to_s=tt(got[i + lag]),
                          what="something appears" if kind == "appears" else "something leaves",
                          frames=fs, pictures=[_jpeg(_thumb(clip_imgs[f], 96)) for f in fs]))

    # runs of "looks different from usual", and the stretch (different or usual) holding this frame
    runs, idx_runs = [], _runs(changed)
    for a, b in idx_runs:
        fa, fb = got[a], got[b]
        runs.append(dict(frame_from=fa, frame_to=fb, time_s=tt(fa), time_to_s=tt(fb),
                         seconds=round(tt(fb) - tt(fa), 2), peak=round(float(change[a:b + 1].max()), 1)))
    i_now = got.index(int(frame_now)) if frame_now is not None and int(frame_now) in patches else None
    now_changed = bool(i_now is not None and changed[i_now])
    frac = float(changed.mean())
    stretch = None
    if i_now is not None:
        if now_changed:
            stretch = next(((a, b) for a, b in idx_runs if a <= i_now <= b), (i_now, i_now))
        else:
            a = max([b + 1 for _, b in idx_runs if b < i_now] + [0])
            b = min([a2 - 1 for a2, _ in idx_runs if a2 > i_now] + [len(got) - 1])
            stretch = (a, b)
    moves = []
    if stretch is not None and guesses and (now_changed or idx_runs):
        a, b = stretch
        lo, hi = got[a], got[b]
        dt = (times[1] - times[0]) if times and len(times) > 1 else 1 / 30.0
        pad = int(round(5.0 / max(1e-6, dt)))
        # the frames just outside the stretch (up to the neighbouring samples, plus 5 s)
        w_lo = got[a - 1] if a > 0 else lo
        w_hi = got[b + 1] if b + 1 < len(got) else hi
        window = range(max(0, w_lo - pad), min(n_frames, w_hi + pad + 1))
        moves = _track_moves(guesses, window, spot.x, spot.y, spot.r, lo, hi,
                             times or [f / 30.0 for f in range(n_frames)])
    sa, sb = (tt(got[stretch[0]]), tt(got[stretch[1]])) if stretch else (0.0, 0.0)

    pct_usual = int(round(100 * (1 - frac)))
    if moves:
        kind = "moves"
        headline = "Something is here from %.1f s to %.1f s that moves: an object." % (sa, sb)
        detail = ("The guesses followed it on or off this spot; jump to see where it came from and where it went."
                  + ("" if now_changed else " It stays here most of the recording, which is why the spot "
                                            "usually looks like this."))
    elif not now_changed and frac < 0.02:
        kind = "background"
        headline = "This spot looks the same through the whole recording."
        detail = ("It is most likely part of the background (a mark, a stain, a fixture), or something that never "
                  "moved. A label here is probably not an object you want. \"Ignore this spot\" leaves guesses here "
                  "out from now on.")
    elif not now_changed:
        kind = "background_visited"
        headline = "On this frame the spot looks as it usually does (%d%% of the recording)." % pct_usual
        detail = ("Something different was there %d time%s (listed below): an object passing over or stopping. "
                  "What is here on this frame is most likely part of the background. Ignoring this spot would "
                  "also leave out the objects that pass over it." % (len(runs), "" if len(runs) == 1 else "s"))
    else:
        kind = "appears"
        if sb - sa >= 2.0:
            headline = "Something is here from %.1f s to %.1f s and isn't there the rest of the time." % (sa, sb)
            detail = ("Nothing was seen moving on or off the spot: most likely something put down and taken away "
                      "(or a shadow that stayed). Jump to the moments it appeared and left.")
        else:
            headline = "Something is here only briefly (around %.1f s)." % sa
            detail = "Most likely something passing over the spot (a hand, a shadow), not an object that stays."

    return dict(x=round(spot.x, 1), y=round(spot.y, 1), r=round(spot.r, 1), frame=frame_now, stretch=None if stretch is None else dict(frame_from=got[stretch[0]], frame_to=got[stretch[1]], time_s=sa, time_to_s=sb),
                samples=len(got), threshold=round(thr, 1), share_changed=round(frac, 3),
                kind=kind, headline=headline, detail=detail, now_changed=now_changed,
                line=dict(frames=got, times=[tt(f) for f in got], change=[round(float(c), 1) for c in change]),
                thumbs=[dict(frame=f, time_s=tt(f), changed=bool(changed[got.index(f)]), picture=_jpeg(_thumb(thumbs[f])))
                        for f in sorted(thumbs)],
                background=_jpeg(_thumb(bg_pic, 144)) if bg_pic is not None else "",
                now_picture=_jpeg(_thumb(now_pic["pic"], 144)) if "pic" in now_pic else "",
                clips=clips, runs=runs[:30], n_runs=len(runs), moves=moves,
                crop_half=spot.C)
