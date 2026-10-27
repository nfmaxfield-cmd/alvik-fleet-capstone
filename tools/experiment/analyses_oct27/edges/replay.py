"""replay.py : each border hold and rescue of both stages against the 49fd943 rules (spin_allowance_cm, rescue with no
drive to reverse carries on when the camera has it EDGE_TURN_CM - 0.5 in). Turn sizes from the state's turns_cut."""
import json, re, math, sys, os
from common import STAGE, lt
TURN_SWING_CM, TURN_SPIN_DRIFT_CM, EDGE_TURN_CM = 3.0, 2.5, 9.0


def allow(a):
    return 2.0 * TURN_SWING_CM * math.sin(math.radians(min(abs(a), 180.0)) / 2.0) + TURN_SPIN_DRIFT_CM


for st in ('E1', 'E2'):
    s = json.load(open(os.path.join(STAGE[st], 'e02_state.json')))
    cuts = s.get('turns_cut', [])
    print('==', st)
    for h in s.get('holds', []):
        t, robot, kind, why = h[:4]
        m = re.search(r'moved ([\d.]+) cm .* while it turned .*now (\d+) cm', why)
        if m:
            mv, d = float(m.group(1)), float(m.group(2))
            c = min((c for c in cuts if c[1] == robot), key=lambda c: abs(c[0] - t), default=None)
            a = c[2] if c and abs(c[0] - t) < 3 else 180.0
            lim = allow(a) if d >= EDGE_TURN_CM else TURN_SPIN_DRIFT_CM
            print('%s %-6s turn %6.1f deg: moved %.1f cm, %2.0f cm from the edge; new limit %.2f -> %s; outcome %s' % (
                lt(t), robot, a, mv, d, lim, 'still holds' if mv > lim else 'NO HOLD', h[5]))
        else:
            print('%s %-6s %s -> not a turn hold (49fd943 changes nothing); outcome %s' % (lt(t), robot, why, h[5]))
    for r in s.get('rescues', []):
        t, robot, kind, why, ok, steps = r[:6]
        room = next((float(re.search(r'([-\d.]+) cm', x).group(1)) for x in steps if x.startswith('the camera')), None)
        nodrive = any('no drive known' in x for x in steps)
        new = ok or (nodrive and room is not None and room >= EDGE_TURN_CM - 0.5 and kind == 'border')
        print('  rescue %s %-6s ok=%s steps=%s -> with 49fd943: %s (margin %s cm)' % (
            lt(t), robot, ok, steps, 'carries on' if new else 'left out',
            '%.1f' % (room - (EDGE_TURN_CM - 0.5)) if room is not None else '-'))
