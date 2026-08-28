#!/usr/bin/env python3
"""
Plots the real extrude-F distribution from a working reference print and
compares the 4 rescale functions considered for mapping it into the
hotend-safe target range: linear, log-scale, gamma/power-law, and
percentile-rank (the one actually implemented in trajectory.cpp).

Data source: roslog_mini_cube_pre_batch.txt (the pre-batch/unbatched
architecture's real per-point G1 E/F commands, used as the "raw" reference
distribution since that architecture never rescales -- its numbers are
whatever the real geometry/timing naturally produced).
"""
import re
import numpy as np
import matplotlib.pyplot as plt

LOG_PATH = "/home/rishika/ws/meca500/src/Final_Project/roslog_mini_cube_pre_batch.txt"
TMIN, TMAX = 17.5, 175.0  # current target range (mm/min)


def load_extrude_f(path):
    pattern = re.compile(r'send_ender\("G1 E(-?[0-9.]+) F([0-9.]+)"\)')
    vals = []
    with open(path, errors="ignore") as f:
        for line in f:
            m = pattern.search(line)
            if m:
                e, fv = float(m.group(1)), float(m.group(2))
                if e > 0:
                    vals.append(fv)
    return np.array(sorted(vals))


def linear(raw, rmin, rmax):
    t = (raw - rmin) / (rmax - rmin)
    return TMIN + (TMAX - TMIN) * t


def log_scale(raw, rmin, rmax):
    t = (np.log(raw) - np.log(rmin)) / (np.log(rmax) - np.log(rmin))
    return TMIN + (TMAX - TMIN) * t


def gamma(raw, rmin, rmax, g):
    t = (raw - rmin) / (rmax - rmin)
    return TMIN + (TMAX - TMIN) * np.power(t, g)


def percentile_rank(raw, sorted_raw):
    n = len(sorted_raw)
    ranks = np.searchsorted(sorted_raw, raw, side="right") / n
    return TMIN + (TMAX - TMIN) * ranks


def main():
    raw = load_extrude_f(LOG_PATH)
    rmin, rmax = raw.min(), raw.max()
    n = len(raw)
    print(f"Loaded {n} real extrude-F samples, range [{rmin:.2f}, {rmax:.2f}] mm/min")

    candidates = {
        "linear (baseline)": lambda r: linear(r, rmin, rmax),
        "log-scale": lambda r: log_scale(r, rmin, rmax),
        "gamma=0.4": lambda r: gamma(r, rmin, rmax, 0.4),
        "gamma=0.25": lambda r: gamma(r, rmin, rmax, 0.25),
        "percentile-rank (implemented)": lambda r: percentile_rank(r, raw),
    }
    colors = {
        "linear (baseline)": "tab:gray",
        "log-scale": "tab:orange",
        "gamma=0.4": "tab:green",
        "gamma=0.25": "tab:red",
        "percentile-rank (implemented)": "tab:blue",
    }

    fig, axes = plt.subplots(3, 2, figsize=(13, 14))
    fig.suptitle(
        f"Extrude-rate rescale comparison (n={n} real samples, "
        f"raw range [{rmin:.2f}, {rmax:.2f}] -> target [{TMIN}, {TMAX}] mm/min)",
        fontsize=13,
    )

    # 1. Raw distribution histogram
    ax = axes[0][0]
    ax.hist(raw, bins=40, color="tab:blue", edgecolor="black", alpha=0.8)
    ax.set_title("Raw distribution (real, unrescaled)")
    ax.set_xlabel("raw matched_f (mm/min)")
    ax.set_ylabel("count")

    # 2. Raw distribution on log x-axis, to show the 3-cluster structure clearly
    ax = axes[0][1]
    ax.hist(raw, bins=np.logspace(np.log10(rmin), np.log10(rmax), 40),
             color="tab:blue", edgecolor="black", alpha=0.8)
    ax.set_xscale("log")
    ax.set_title("Same raw distribution, log-x (shows the 3 clusters)")
    ax.set_xlabel("raw matched_f (mm/min, log scale)")
    ax.set_ylabel("count")

    # 3. Mapping curves: raw value -> mapped value, for every candidate
    ax = axes[1][0]
    raw_sorted = np.sort(raw)
    for name, fn in candidates.items():
        ax.plot(raw_sorted, fn(raw_sorted), label=name, color=colors[name], linewidth=1.5)
    ax.set_title("Mapping curve: raw -> target value")
    ax.set_xlabel("raw matched_f (mm/min)")
    ax.set_ylabel("mapped matched_f (mm/min)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # 4. Same mapping curves, raw x on log scale (clearer near the dense low end)
    ax = axes[1][1]
    for name, fn in candidates.items():
        ax.plot(raw_sorted, fn(raw_sorted), label=name, color=colors[name], linewidth=1.5)
    ax.set_xscale("log")
    ax.set_title("Same mapping curves, log-x")
    ax.set_xlabel("raw matched_f (mm/min, log scale)")
    ax.set_ylabel("mapped matched_f (mm/min)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # 5. Resulting mapped distributions, overlaid histograms
    ax = axes[2][0]
    for name, fn in candidates.items():
        mapped = fn(raw)
        ax.hist(mapped, bins=40, range=(TMIN, TMAX), histtype="step",
                 label=name, color=colors[name], linewidth=1.8)
    ax.set_title("Resulting mapped distributions (overlaid)")
    ax.set_xlabel("mapped matched_f (mm/min)")
    ax.set_ylabel("count")
    ax.legend(fontsize=8)

    # 6. Percentile table as text, for the numbers behind the plots
    ax = axes[2][1]
    ax.axis("off")
    pct_points = [0, 10, 25, 50, 75, 90, 100]
    rows = [["function"] + [f"p{p}" for p in pct_points]]
    for name, fn in candidates.items():
        mapped = fn(raw)
        pct = np.percentile(mapped, pct_points)
        rows.append([name] + [f"{v:.1f}" for v in pct])
    table = ax.table(cellText=rows, loc="center", cellLoc="center",
                       colWidths=[0.32] + [0.10] * len(pct_points))
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    table.scale(1, 1.6)
    ax.set_title("Resulting percentiles per function", pad=20)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    out_path = "rescale_comparison.png"
    plt.savefig(out_path, dpi=130)
    print(f"Saved plot -> {out_path}")


if __name__ == "__main__":
    main()
