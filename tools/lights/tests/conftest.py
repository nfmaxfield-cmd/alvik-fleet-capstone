import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
LIGHTS = os.path.dirname(HERE)
TOOLS = os.path.dirname(LIGHTS)
for p in (LIGHTS, TOOLS, os.path.join(TOOLS, "sim"), os.path.join(TOOLS, "live"), os.path.join(TOOLS, "blink"),
          os.path.join(TOOLS, "blink", "tests"), os.path.join(TOOLS, "fusion")):
    if p not in sys.path:
        sys.path.insert(0, p)
