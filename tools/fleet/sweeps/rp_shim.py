#!/usr/bin/env python3
"""run_plan.py, unchanged, with its simulator swapped for sim_shim.py (the school's conditions), for the Monday sweep.
Meant to run in its own network namespace (`unshare -n`): it brings the namespace's loopback up first, so the
simulated robots' ports (5005, 5006, 5010) never meet another dry run on this machine.

    unshare -n python3 tools/fleet/sweeps/rp_shim.py <run_plan.py arguments>
"""
import fcntl
import os
import socket
import struct
import subprocess
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
FLEET = os.path.dirname(HERE)
SHIM = os.path.join(HERE, "sim_shim.py")
if FLEET not in sys.path:
    sys.path.insert(0, FLEET)


def loopback_up():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        ifr = struct.pack("16sH14s", b"lo", 0, b"\0" * 14)
        flags = struct.unpack("16sH14s", fcntl.ioctl(s, 0x8913, ifr))[1]          # SIOCGIFFLAGS
        if not flags & 1:
            fcntl.ioctl(s, 0x8914, struct.pack("16sH14s", b"lo", flags | 1, b"\0" * 14))   # SIOCSIFFLAGS: IFF_UP
    finally:
        s.close()


def main():
    loopback_up()
    import run_plan as RP

    def popen(cmd, *a, **k):
        if isinstance(cmd, list) and len(cmd) > 1 and cmd[1].endswith(os.path.join("sim", "fleet_sim.py")):
            cmd = [cmd[0], SHIM] + list(cmd[2:])
        return subprocess.Popen(cmd, *a, **k)
    proxy = types.ModuleType("subprocess")
    proxy.__dict__.update(subprocess.__dict__)
    proxy.Popen = popen
    RP.subprocess = proxy
    return RP.main(sys.argv[1:])


if __name__ == "__main__":
    sys.exit(main())
