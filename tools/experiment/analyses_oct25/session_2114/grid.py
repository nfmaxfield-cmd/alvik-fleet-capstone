"""grid.py IN OUT x0 x1 y0 y1 zoom : y = 6, 4, ... -8 cm lines (at the robot's top height, 3.7 cm) and x = 0, 4, 8, 12
across the bottom-right corner, labelled, for reading a body's extent by eye."""
import sys, cv2, numpy as np
import pose, e02_explore as EX
im = cv2.imread(sys.argv[1])
x0, x1, y0, y1, z = (int(v) for v in sys.argv[3:8])
im = cv2.resize(im[y0:y1, x0:x1], None, fx=z, fy=z, interpolation=cv2.INTER_CUBIC)
def P(x, y, h):
    u, v = pose.ROAM.project((x, y, h))
    return int(round((u - x0) * z)), int(round((v - y0) * z))
for yy in range(6, -10, -2):
    col = (0, 0, 255) if yy == 0 else (255, 120, 0) if yy < 0 else (0, 200, 0)
    cv2.line(im, P(-4, yy, EX.ROBOT_TOP_CM), P(20, yy, EX.ROBOT_TOP_CM), col, 1, cv2.LINE_AA)
    cv2.putText(im, str(yy), P(19, yy, EX.ROBOT_TOP_CM), cv2.FONT_HERSHEY_SIMPLEX, 0.35, col, 1)
for xx in (0, 4, 8, 12):
    col = (0, 0, 255) if xx == 0 else (0, 255, 255)
    cv2.line(im, P(xx, 8, EX.ROBOT_TOP_CM), P(xx, -10, EX.ROBOT_TOP_CM), col, 1, cv2.LINE_AA)
cv2.imwrite(sys.argv[2], im)
