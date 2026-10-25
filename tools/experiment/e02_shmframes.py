"""Camera pictures shared between the tracker process and the runner without copying them through a pipe
(2026-10-25 evening, MIRLT): a ring of picture slots in shared memory (multiprocessing.shared_memory).

The tracker process writes every decoded picture into the next slot (the oldest is overwritten) and tells the runner
its number over the pipe (a few bytes); the runner reads the picture straight out of the shared memory when it needs
it (the live page, the naming check, finds by a turn). A picture is never pickled.

Layout of the block: a header (int64: magic, slots, height, width, newest number), then per slot a row of float64
(number, picture time, Pi time, recorder frame number, arrival time), then the pictures (uint8, height x width each).
A slot's number is set to -1 while it is being written and to the picture's number when it is complete, so a reader
that finds a different number before or after its copy knows the slot was overwritten meanwhile and gets None (a
seqlock). The writer is one thread of one process.

    ring = FrameRing.create(slots=128, height=480, width=640)      # the tracker process (it owns the block)
    n = ring.write(img, t, t_pi=..., pi_frame=..., t_arrived=...)
    ring2 = FrameRing.attach(ring.name)                            # the runner
    img, meta = ring2.read(n)                                      # a copy, or None once overwritten
"""
import time

import numpy as np

MAGIC = 0x45303246524D4553        # "SEMRF20E"
HEAD_N = 8                        # int64 header fields
META_N = 5                        # float64 per slot: number, t, t_pi, pi_frame, t_arrived
SLOTS = 128                       # chosen: 5 s at 25 pictures a second (collect() asks for up to 3.6 s; finds by a
                                  # turn wait for 8 + 8). 128 x 640 x 480 = 39 MB. No DOE: a size, set by what reads it


def _shm():
    from multiprocessing import shared_memory
    return shared_memory


def _untrack(shm):
    """Python 3.8-3.12 register every attached block with this process's resource tracker, which then deletes it
    when this process ends (the owner's job). Undo that for a block this process only attached to."""
    try:
        from multiprocessing import resource_tracker
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception:
        pass


class FrameRing:
    def __init__(self, shm, owner):
        self.shm = shm
        self.owner = owner
        head = np.ndarray((HEAD_N,), np.int64, shm.buf, 0)
        if int(head[0]) != MAGIC:
            raise ValueError("not a picture ring (%s)" % shm.name)
        self.head = head
        self.slots, self.h, self.w = int(head[1]), int(head[2]), int(head[3])
        off = HEAD_N * 8
        self.meta = np.ndarray((self.slots, META_N), np.float64, shm.buf, off)
        off += self.slots * META_N * 8
        self.data = np.ndarray((self.slots, self.h, self.w), np.uint8, shm.buf, off)
        self.name = shm.name

    @staticmethod
    def size_for(slots, height, width):
        return HEAD_N * 8 + slots * META_N * 8 + slots * height * width

    @classmethod
    def create(cls, slots=SLOTS, height=480, width=640, name=None):
        sm = _shm().SharedMemory(name=name, create=True, size=cls.size_for(slots, height, width))
        head = np.ndarray((HEAD_N,), np.int64, sm.buf, 0)
        head[:] = 0
        head[1], head[2], head[3] = slots, height, width
        head[4] = -1
        meta = np.ndarray((slots, META_N), np.float64, sm.buf, HEAD_N * 8)
        meta[:] = -1.0
        head[0] = MAGIC
        return cls(sm, owner=True)

    @classmethod
    def attach(cls, name):
        sm = _shm().SharedMemory(name=name, create=False)
        _untrack(sm)
        return cls(sm, owner=False)

    # --- writer -------------------------------------------------------------------------------------------------
    def write(self, img, t, t_pi=float("nan"), pi_frame=-1, t_arrived=float("nan")):
        """Store a grey picture (height x width uint8); returns its number (0, 1, 2 ...)."""
        if img.shape != (self.h, self.w):
            raise ValueError("picture %s, ring %dx%d" % (img.shape, self.h, self.w))
        n = int(self.head[4]) + 1
        k = n % self.slots
        m = self.meta[k]
        m[0] = -1.0
        self.data[k][...] = img
        m[1], m[2], m[3], m[4] = t, t_pi, -1 if pi_frame is None else pi_frame, t_arrived
        m[0] = float(n)
        self.head[4] = n
        return n

    # --- readers ------------------------------------------------------------------------------------------------
    def newest(self):
        """The newest picture's number, or -1 before the first."""
        return int(self.head[4])

    def meta_of(self, n):
        """(t, t_pi, pi_frame, t_arrived) of picture n, or None if its slot holds another picture now."""
        m = self.meta[n % self.slots]
        if int(m[0]) != n:
            return None
        out = (float(m[1]), float(m[2]), int(m[3]), float(m[4]))
        return out if int(m[0]) == n else None

    def read(self, n, out=None):
        """A copy of picture n and its (t, t_pi, pi_frame, t_arrived), or None if it was overwritten (or not yet
        written)."""
        if n < 0:
            return None
        k = n % self.slots
        m = self.meta[k]
        if int(m[0]) != n:
            return None
        meta = (float(m[1]), float(m[2]), int(m[3]), float(m[4]))
        img = np.array(self.data[k], copy=True) if out is None else np.copyto(out, self.data[k]) or out
        if int(m[0]) != n:
            return None
        return img, meta

    def since(self, t_from, t_to=None):
        """Numbers of the pictures still in the ring with t_from <= t (<= t_to), oldest first."""
        newest = self.newest()
        out = []
        for n in range(max(0, newest - self.slots + 1), newest + 1):
            mm = self.meta_of(n)
            if mm is not None and mm[0] >= t_from and (t_to is None or mm[0] <= t_to):
                out.append(n)
        return out

    def close(self):
        try:
            # drop the numpy views first (a block with views still on it can't be closed)
            self.head = self.meta = self.data = None
            self.shm.close()
            if self.owner:
                self.shm.unlink()
        except Exception:
            pass


def wait_for(fn, timeout, step=0.005):
    """Polls fn() until it returns something true or timeout s pass (tests and the bench)."""
    end = time.time() + timeout
    while time.time() < end:
        v = fn()
        if v:
            return v
        time.sleep(step)
    return fn()
