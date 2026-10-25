import sys, csv, cv2, bisect
V = '/tmp/claude-0/s1025/vid/cam0_video.avi.part-aa'
F = '/tmp/claude-0/s1025/rec/20261025-150957_E02/cam0/frames.csv'
tw = [float(r['t_wall']) for r in csv.DictReader(open(F))]
def ep(s):
    h, m, sec = s.split(':'); return 1792955404.0 + (int(h)*3600 + int(m)*60 + float(sec)) - (15*3600 + 10*60 + 4)
cap = cv2.VideoCapture(V)
print('frames in file', cap.get(cv2.CAP_PROP_FRAME_COUNT), 'csv', len(tw))
for s in sys.argv[2:]:
    i = bisect.bisect_left(tw, ep(s))
    cap.set(cv2.CAP_PROP_POS_FRAMES, i)
    ok, img = cap.read()
    if ok:
        out = '%s/f_%s.png' % (sys.argv[1], s.replace(':', ''))
        cv2.imwrite(out, img); print(s, i, out)
    else:
        print(s, i, 'read failed')
