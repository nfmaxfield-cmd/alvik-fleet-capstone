#!/usr/bin/env python3
"""Cut a big file (a session video) into pieces under GitHub's 2 GB release limit, without changing a byte:
video.avi -> video.part-aa, video.part-ab, ... (put back together: the pieces one after another; the replay tools
also read a piece on its own: e02_replay.MjpgPiece).

    python tools/split_file.py recordings_oct23\\video.avi
    python tools/split_file.py recordings_oct23\\video.avi --mb 1900
"""
import argparse
import os
import string


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--mb", type=int, default=1900, help="piece size in MB (GitHub's limit is 2000)")
    a = ap.parse_args()
    size = a.mb * 1024 * 1024
    stem = os.path.splitext(a.path)[0]
    names = [x + y for x in string.ascii_lowercase for y in string.ascii_lowercase]
    total = os.path.getsize(a.path)
    with open(a.path, "rb") as f:
        k = 0
        while True:
            out = "%s.part-%s" % (stem, names[k])
            left = size
            wrote = 0
            with open(out, "wb") as g:
                while left > 0:
                    b = f.read(min(left, 8 * 1024 * 1024))
                    if not b:
                        break
                    g.write(b)
                    left -= len(b)
                    wrote += len(b)
            if wrote == 0:
                os.remove(out)
                break
            print("wrote %s (%.0f MB)" % (out, wrote / 1e6))
            k += 1
            if f.tell() >= total:
                break
    print("done: %d pieces from %s (%.2f GB)" % (k, a.path, total / 1e9))


if __name__ == "__main__":
    main()
