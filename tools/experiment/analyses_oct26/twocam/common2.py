import sys, os
REPO = '/home/claude/alvik-fleet-capstone'
sys.path.insert(0, REPO + '/tools/experiment'); sys.path.insert(0, REPO + '/tools/recognition')
import numpy as np, cv2
import e02_camera as CAM
OCAM = CAM.load_camera(REPO + '/tools/experiment/data/overhead_floorcal_roam1.json')
COL = {'Left': (60, 200, 255), 'Center': (80, 255, 80), 'Right': (255, 120, 255)}
def dec(b): return cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR)
