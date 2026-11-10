#!/usr/bin/env python3
"""Desk simulator for main.py: runs the robot program, unchanged, against
simulated obstacles and reports how close to the start it ended.

Examples (from the repo folder):
  python3 sim/run.py                        # current main.py, all shapes
  python3 sim/run.py d_baseline s2_slant_ese_lag
  python3 sim/run.py --rev 356d010 --rev HEAD --sonar wide
  python3 sim/run.py --code main_working_v3.py --plot

See sim/README.md for what the columns mean and what the model leaves out.
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "fake"))
sys.path.insert(0, HERE)
from pybricks._world import SONAR_MODELS, distance_to  # noqa: E402
from scenarios import DEFAULT, SCENARIOS  # noqa: E402

START_TIP = (2000.0, 500.0)    # measuring point at the start
START_AXLE = (2006.0, 394.0)   # axle at the start
TIP_F, TIP_L = 106.0, 6.0
LAP_ENDS = ("Front wall ended", "Back at hit point", "Wall end not found", "Bumped while",
            "Turned past", "Out of time", "STOPPED")


def program_for(args):
    # Each version to run: (label, path to its main.py).
    versions = []
    for rev in args.rev:
        path = os.path.join(args.out, "programs", "main_%s.py" % re.sub(r"\W", "_", rev))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write(subprocess.check_output(["git", "-C", REPO, "show", "%s:main.py" % rev], text=True))
        versions.append((rev, path))
    for code in args.code:
        versions.append((os.path.splitext(os.path.basename(code))[0], os.path.abspath(code)))
    return versions or [("main", os.path.join(REPO, "main.py"))]


def measure(workdir, scenario):
    # Read one finished run and work out how it went.
    polygons = SCENARIOS[scenario][0]
    result = json.load(open(os.path.join(workdir, "result.json")))
    lines = open(os.path.join(workdir, "console.txt")).read().splitlines()
    truth = list(csv.DictReader(open(os.path.join(workdir, "truth.csv"))))
    last = truth[-1]
    tx, ty, heading = float(last["true_px"]), float(last["true_py"]), float(last["true_heading"])
    h = math.radians(heading)
    ax, ay = tx - TIP_F * math.sin(h) + TIP_L * math.cos(h), ty - TIP_F * math.cos(h) - TIP_L * math.sin(h)
    row = {
        "body": math.hypot(ax - START_AXLE[0], ay - START_AXLE[1]),
        "tip": math.hypot(tx - START_TIP[0], ty - START_TIP[1]),
        "heading": (heading + 180.0) % 360.0 - 180.0,
        "bumps": sum(1 for line in lines if line.startswith("BUMP")),
        "ended": next((l for l in reversed(lines) if l.startswith(LAP_ENDS)), "?"),
        "task_s": None, "wall_max": None, "over_30": None,
    }
    if result["crashed"]:
        row["ended"] = "CRASHED: " + result["crashed"].strip().splitlines()[-1]
    done = [l for l in lines if l.startswith("RESULT")]
    if done:
        row["task_s"] = int(re.search(r"at (\d+) ms", done[0]).group(1)) / 1000.0
    # 30 cm rule while following: needs the program's own run log for timing.
    logs = glob.glob(os.path.join(workdir, "runs", "run_*.csv"))
    if logs and polygons:
        est = list(csv.DictReader(open(logs[0])))
        offset = float(last["t_ms"]) - int(est[-1]["t_ms"])  # sim clock - task clock
        follow = [int(r["t_ms"]) + offset for r in est if r["phase"] in ("FOLLOW", "BUMP")]
        if follow:
            t0, t1 = min(follow) + 3000, max(follow)  # skip settling after the first turn
            gaps = [distance_to(float(r["true_px"]), float(r["true_py"]), polygons)
                    for r in truth if t0 <= float(r["t_ms"]) <= t1]
            if gaps:
                row["wall_max"] = max(gaps)
                row["over_30"] = sum(1 for g in gaps if g > 300) * 0.05
    return row


def fmt(value, pattern):
    return "-" if value is None else pattern % value


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("scenarios", nargs="*", help="scenario names (default: all but e_no_obstacle)")
    p.add_argument("--code", action="append", default=[], help="a robot program to run (repeatable)")
    p.add_argument("--rev", action="append", default=[], help="run main.py from this git commit (repeatable)")
    p.add_argument("--sonar", choices=sorted(SONAR_MODELS), default="medium",
                   help="sonar beam model (default medium)")
    p.add_argument("--out", default=os.path.join(HERE, "out"), help="where runs are saved (default sim/out)")
    p.add_argument("--plot", action="store_true", help="also save a picture of the paths (needs matplotlib)")
    p.add_argument("--list", action="store_true", help="list the scenarios and exit")
    args = p.parse_args()
    if args.list:
        for name, (_, _, about) in SCENARIOS.items():
            print("%-28s %s" % (name, about))
        return
    names = args.scenarios or DEFAULT
    for name in names:
        if name not in SCENARIOS:
            sys.exit("Unknown scenario %r (see --list)" % name)
    report = []
    for label, program in program_for(args):
        report.append("== %s   (sonar: %s)" % (label, args.sonar))
        report.append("%-27s %5s %5s %5s %6s %6s %5s %6s  %s" % (
            "scenario", "body", "tip", "head", "wall", ">30cm", "bumps", "time", "how the lap ended"))
        report.append("%-27s %5s %5s %5s %6s %6s %5s %6s" % ("", "mm", "mm", "deg", "max mm", "s", "", "s"))
        print("\n".join(report[-3:]), flush=True)
        bodies = []
        for name in names:
            workdir = os.path.join(args.out, "%s_%s" % (label, args.sonar), name)
            subprocess.run([sys.executable, os.path.join(HERE, "run_one.py"), program, name,
                            args.sonar, workdir], check=True)
            r = measure(workdir, name)
            bodies.append(r["body"])
            line = "%-27s %5d %5d %5d %6s %6s %5d %6s  %s" % (
                name, r["body"], r["tip"], r["heading"], fmt(r["wall_max"], "%d"),
                fmt(r["over_30"], "%.1f"), r["bumps"], fmt(r["task_s"], "%.1f"), r["ended"][:70])
            report.append(line)
            print(line, flush=True)
        ranked = sorted(bodies)
        line = "%d runs: median %d mm, worst %d mm, %d more than 15 cm off" % (
            len(ranked), ranked[len(ranked) // 2], ranked[-1], sum(1 for b in ranked if b > 150))
        report += [line, ""]
        print(line + "\n", flush=True)
        if args.plot:
            from plot import plot
            out = os.path.join(args.out, "%s_%s" % (label, args.sonar), "paths.png")
            plot([os.path.join(args.out, "%s_%s" % (label, args.sonar), n) for n in names], out)
            print("Paths saved to", out, "\n")
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "summary_%s.txt" % args.sonar), "w") as f:
        f.write("\n".join(report) + "\n")


if __name__ == "__main__":
    main()
