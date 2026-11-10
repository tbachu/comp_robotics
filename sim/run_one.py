# Run one copy of a robot program in one scenario. Used by run.py, which
# starts a fresh Python for each run so module state never carries over.
# Usage: python3 run_one.py PROGRAM.py SCENARIO SONAR WORKDIR
import contextlib
import json
import os
import runpy
import shutil
import sys
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "fake"))
sys.path.insert(0, HERE)
from pybricks._world import WORLD  # noqa: E402
from scenarios import SCENARIOS  # noqa: E402

program, scenario, sonar, workdir = sys.argv[1:5]
polygons, slip, _ = SCENARIOS[scenario]
WORLD.reset(polygons, slip, sonar)
if os.path.isdir(workdir):
    shutil.rmtree(workdir)
os.makedirs(workdir)
# A copy, so the program's run logs land in WORKDIR/runs, not next to main.py.
copy = os.path.join(workdir, "main.py")
shutil.copy(program, copy)
crashed = None
with open(os.path.join(workdir, "console.txt"), "w") as out, contextlib.redirect_stdout(out):
    try:
        runpy.run_path(copy, run_name="__main__")
    except BaseException:
        crashed = traceback.format_exc()
        print(crashed)
with open(os.path.join(workdir, "truth.csv"), "w") as f:
    f.write("t_ms,true_px,true_py,true_heading\n")
    for t, x, y, h in WORLD.trace:
        f.write("%d,%.1f,%.1f,%.1f\n" % (t, x, y, h))
json.dump({"sim_ms": WORLD.t, "crashed": crashed}, open(os.path.join(workdir, "result.json"), "w"))
