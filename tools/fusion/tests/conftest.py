"""Make tools/fusion and these tests' helpers importable."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FUSION = os.path.dirname(HERE)
for p in (HERE, FUSION, os.path.dirname(FUSION)):
    if p not in sys.path:
        sys.path.insert(0, p)
