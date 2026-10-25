import sys
sys.path[:0] = ['/home/claude/alvik-fleet-capstone/tools/recognition']
import numpy as np, cv2
import cameras as CMS
cam = CMS.CameraModel.load('/home/claude/alvik-fleet-capstone/tools/experiment/data/overhead_floorcal_roam1.json')
g = cv2.GaussianBlur(cv2.cvtColor(cv2.imread('/tmp/claude-0/s1025/fr/f_151020.png'), cv2.COLOR_BGR2GRAY).astype(float), (3, 3), 0)
W, H = 122.15, 121.9
P = lambda x, y: np.array(cam.project((x, y, 0.0)), float).ravel()
sides = {'right/open y=0': ((0,0),(W,0),(0,1)), 'bottom/open x=0': ((0,0),(0,H),(1,0)), 'left/board y=H': ((0,H),(W,H),(0,-1)), 'top/board x=W': ((W,0),(W,H),(-1,0))}
for name,(a,b,n) in sides.items():
    a=np.array(a,float); b=np.array(b,float); n=np.array(n,float)
    for f in (0.3,0.5,0.7):
        p=a+f*(b-a); s=[]
        for d in np.arange(-10,12.01,1.0):
            u,v=P(*(p+d*n))
            s.append('%3d'%g[int(round(v)),int(round(u))] if 0<=u<640 and 0<=v<480 else '  .')
        print('%-16s f=%.1f  d=-10..+11cm (inward +):'%(name,f),' '.join(s))
