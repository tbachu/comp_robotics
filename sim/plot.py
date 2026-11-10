# Draw the true paths of finished runs over their obstacles (needs matplotlib).
# Usage: python3 sim/plot.py OUT.png RUNDIR...   (RUNDIR = sim/out/<version>_<sonar>/<scenario>)
import csv
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from scenarios import SCENARIOS  # noqa: E402


def plot(rundirs, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cols = min(4, len(rundirs))
    rows = (len(rundirs) + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(4.5 * cols, 4.8 * rows), squeeze=False)
    for ax in axes.flat:
        ax.axis("off")
    for ax, rundir in zip(axes.flat, rundirs):
        name = os.path.basename(rundir.rstrip("/"))
        ax.axis("on")
        for poly in SCENARIOS[name][0]:
            ax.fill([q[0] for q in poly], [q[1] for q in poly], color="#bbbbbb")
        truth = list(csv.DictReader(open(os.path.join(rundir, "truth.csv"))))
        ax.plot([float(r["true_px"]) for r in truth], [float(r["true_py"]) for r in truth],
                lw=1, color="#1f6feb")
        ax.plot(2000, 500, "k*", ms=10)
        ax.set_aspect("equal")
        ax.set_title(name, fontsize=9)
        ax.set_xlim(900, 3300)
        ax.set_ylim(300, 3100)
    plt.tight_layout()
    plt.savefig(out, dpi=80)


if __name__ == "__main__":
    plot(sys.argv[2:], sys.argv[1])
