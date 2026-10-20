#!/usr/bin/env python3
"""One command to install what the recognition studio needs and start it.

    python tools/recognition/start.py                        uses the project folder "recognition-project" in your Documents
    python tools/recognition/start.py <project folder>       uses (or creates) that project
    python tools/recognition/start.py <project folder> --add <recording folder>

The first time, it installs the missing Python packages (numpy, opencv-python,
onnxruntime, onnx, and PyTorch for training; PyTorch is the large one, ~200 MB, and
the CPU version is chosen). Later starts skip that and open the studio straight away.
Everything else is passed on to studio.py (see its --help).
"""
import importlib
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

NEEDED = [  # (import name, pip name, extra pip arguments)
    ("numpy", "numpy", []),
    ("cv2", "opencv-python", []),
    ("onnxruntime", "onnxruntime", []),
    ("onnx", "onnx", []),
    ("torch", "torch", ["--index-url", "https://download.pytorch.org/whl/cpu"]),
]


def missing():
    out = []
    for mod, pip, extra in NEEDED:
        try:
            importlib.import_module(mod)
        except Exception:
            out.append((mod, pip, extra))
    return out


def install(items):
    plain = [p for _, p, e in items if not e]
    if plain:
        print("Installing: " + ", ".join(plain))
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--upgrade"] + plain)
    for _, pip, extra in items:
        if extra:
            print("Installing %s (the large one; a few minutes) ..." % pip)
            try:
                subprocess.check_call([sys.executable, "-m", "pip", "install", pip] + extra)
            except subprocess.CalledProcessError:
                print("The CPU-only download failed; trying the standard one ...")
                subprocess.check_call([sys.executable, "-m", "pip", "install", pip])


def default_project():
    docs = os.path.join(os.path.expanduser("~"), "Documents")
    return os.path.join(docs if os.path.isdir(docs) else os.path.expanduser("~"), "recognition-project")


def main():
    if sys.version_info < (3, 9):
        sys.exit("Python 3.9 or newer is needed (this is %d.%d). Install it from python.org." % sys.version_info[:2])
    todo = missing()
    if todo:
        print("First start: some packages are missing (%s)." % ", ".join(p for _, p, _ in todo))
        try:
            install(todo)
        except subprocess.CalledProcessError as e:
            sys.exit("Installing failed (%s). Check the internet connection and try again." % e)
        still = missing()
        if still:
            sys.exit("Still missing after installing: %s" % ", ".join(p for _, p, _ in still))
        print("All packages installed.\n")
    args = sys.argv[1:]
    if not args or args[0].startswith("-"):
        args = [default_project()] + args
        print("Project folder: %s" % args[0])
    sys.argv = [os.path.join(HERE, "studio.py")] + args
    sys.path.insert(0, HERE)
    import studio
    return studio.main()


if __name__ == "__main__":
    sys.exit(main())
