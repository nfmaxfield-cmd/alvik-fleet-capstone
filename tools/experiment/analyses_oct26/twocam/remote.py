"""Seekable read-only file over HTTP ranges (joins split parts); used with zipfile and AVI index reading."""
import urllib.request, ssl, os, io
CTX = ssl.create_default_context(cafile='/root/.ccr/ca-bundle.crt')
BASE = 'https://github.com/nfmaxfield-cmd/alvik-fleet-capstone/releases/download/'
_sizes = {}
def get(url, a, b):
    r = urllib.request.Request(url, headers={'Range': 'bytes=%d-%d' % (a, b)})
    for k in range(4):
        try:
            return urllib.request.urlopen(r, context=CTX, timeout=120).read()
        except Exception as e:
            err = e
    raise err
class Remote(io.RawIOBase):
    def __init__(self, urls, sizes, block=1 << 20):
        self.urls, self.sizes = urls, sizes
        self.starts = [sum(sizes[:i]) for i in range(len(sizes))]
        self.n = sum(sizes); self.pos = 0; self.block = block; self.cache = {}
    def readable(self): return True
    def seekable(self): return True
    def tell(self): return self.pos
    def seek(self, o, wh=0):
        self.pos = o if wh == 0 else (self.pos + o if wh == 1 else self.n + o); return self.pos
    def raw(self, a, ln):
        """bytes [a, a+ln) direct, no cache"""
        out = b''; e = min(a + ln, self.n) - 1
        for u, s0, sz in zip(self.urls, self.starts, self.sizes):
            if e < s0 or a > s0 + sz - 1: continue
            out += get(u, max(a, s0) - s0, min(e, s0 + sz - 1) - s0)
        return out
    def read(self, k=-1):
        if k < 0: k = self.n - self.pos
        k = min(k, self.n - self.pos)
        if k <= 0: return b''
        if k > 4 * self.block:
            d = self.raw(self.pos, k); self.pos += len(d); return d
        out = b''
        while len(out) < k:
            bi = (self.pos + len(out)) // self.block
            if bi not in self.cache:
                if len(self.cache) > 64: self.cache.clear()
                self.cache[bi] = self.raw(bi * self.block, self.block)
            blk = self.cache[bi]; o = (self.pos + len(out)) - bi * self.block
            out += blk[o:o + k - len(out)]
        self.pos += len(out); return out
    def readinto(self, b):
        d = self.read(len(b)); b[:len(d)] = d; return len(d)
