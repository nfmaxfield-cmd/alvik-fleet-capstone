"""Side-view robot blobs: per window, a median background of the window's own pictures, then the moving parts."""
import numpy as np, cv2
THR = 28          # chosen: grey-level change that counts as foreground
MIN_AREA = 120    # chosen: px; a robot is 400-1500 px in the side view
def background(imgs):
    return np.median(np.stack(imgs[::max(1, len(imgs) // 25)]), axis=0).astype(np.uint8)
K3 = np.ones((3, 3), np.uint8); K7 = np.ones((7, 7), np.uint8)
def blobs(img, bg):
    d = cv2.absdiff(img, bg).max(axis=2)
    m = (d > THR).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, K3); m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, K7)
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    out = []
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if a >= MIN_AREA:
            out.append(dict(u=float(cen[i][0]), v=float(cen[i][1]), x=int(x), y=int(y), w=int(w), h=int(h), a=int(a)))
    return out, m
