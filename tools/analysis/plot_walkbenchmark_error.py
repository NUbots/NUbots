#!/usr/bin/env python3
"""Plot the tracking error from a WalkPathBenchmark trajectory CSV.

Input: the per-tick CSV WalkPathBenchmark writes to its csv_directory, one row per tick:

    t,trial,x,y,yaw,z,target_x,target_y,target_yaw,err_x,err_y,err_yaw,cmd_vx,cmd_vy,cmd_wz

`t` restarts at zero with each trial, so the trials are laid end to end on one time axis. CSVs written before the
err_* columns existed have them computed from the pose and target, with the yaw error wrapped to [-pi, pi]. The yaw
error is then unwrapped within each trial, so it can run past +-pi rather than jump by 2 pi.

The figures go in a directory beside the CSV named after it, e.g. recordings/walk_path_benchmark/20261020T22_24_30.csv
writes to recordings/walk_path_benchmark/20261020T22_24_30/.

Figures:
    error_over_time.png    err_x and err_y (left axis, m) and err_yaw (right axis, rad) over every trial
    command_over_time.png  cmd_vx and cmd_vy (left axis, m/s) and cmd_wz (right axis, rad/s) over every trial

Usage:
    python3 tools/analysis/plot_walkbenchmark_error.py recordings/walk_path_benchmark/20261020T22_24_30.csv
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import AutoMinorLocator


def load(path: pathlib.Path) -> dict[str, np.ndarray]:
    data = np.genfromtxt(path, delimiter=",", names=True)
    if data.size == 0:
        raise ValueError(f"{path} has no rows")
    data = np.atleast_1d(data)
    columns = {name: data[name] for name in data.dtype.names}
    columns["trial"] = columns["trial"].astype(int)

    # Older CSVs predate the err_* columns
    if "err_x" not in columns:
        columns["err_x"] = columns["target_x"] - columns["x"]
        columns["err_y"] = columns["target_y"] - columns["y"]
        columns["err_yaw"] = np.angle(np.exp(1j * (columns["target_yaw"] - columns["yaw"])))

    columns["err_yaw"] = unwrap_per_trial(columns["err_yaw"], columns["trial"])
    columns["time"] = run_time(columns["t"], columns["trial"])
    return columns


def unwrap_per_trial(angle: np.ndarray, trial: np.ndarray) -> np.ndarray:
    """Makes a wrapped angle continuous within each trial, so an error passing through +-pi (the robot facing away
    from the target heading) does not jump by 2 pi. Each trial is shifted by whole turns so it ends in [-pi, pi]:
    the error it settles to is the shortest one, and the start shows how far the robot really had to turn"""
    out = np.empty_like(angle)
    for index in np.unique(trial):
        rows = trial == index
        unwrapped = np.unwrap(angle[rows])
        out[rows] = unwrapped - 2.0 * np.pi * np.round(unwrapped[-1] / (2.0 * np.pi))
    return out


def run_time(t: np.ndarray, trial: np.ndarray) -> np.ndarray:
    """Time since the first trial started, laying each trial after the end of the previous one"""
    time = np.empty_like(t)
    offset = 0.0
    for index in np.unique(trial):
        rows = trial == index
        time[rows] = offset + t[rows] - t[rows][0]
        # Leave one tick between trials so the end of one does not land on the start of the next
        dt = np.median(np.diff(t[rows])) if rows.sum() > 1 else 0.0
        offset = time[rows][-1] + dt
    return time


def trial_ends(data: dict[str, np.ndarray]) -> list[tuple[int, float]]:
    """(trial, time of its last row) for every trial in the run"""
    return [(int(index), float(data["time"][data["trial"] == index][-1])) for index in np.unique(data["trial"])]


def symmetric_limit(*series: np.ndarray) -> float:
    """Half-height of an axis centred on zero that fits every series, with a little headroom"""
    peak = max(float(np.nanmax(np.abs(s))) for s in series)
    return 1.05 * peak if peak > 0.0 else 1.0


def plot_dual_axis(
    data: dict[str, np.ndarray],
    left: list[tuple[str, str, str]],
    right: tuple[str, str, str],
    left_label: str,
    right_label: str,
    title: str,
    path: pathlib.Path,
) -> pathlib.Path:
    """Plot columns over the run time, `left` as (column, legend label, colour) on the left axis and `right` on the
    right axis, with both axes centred on zero and a dashed line at the end of each trial"""
    fig, ax_left = plt.subplots(figsize=(14, 6))
    ax_right = ax_left.twinx()

    time = data["time"]
    lines = [
        ax_left.plot(time, data[column], color=colour, linewidth=1.2, label=label)[0] for column, label, colour in left
    ]
    column, label, right_colour = right
    lines.append(ax_right.plot(time, data[column], color=right_colour, linewidth=1.2, label=label)[0])

    # Centre both axes on zero so their zero lines coincide
    left_limit = symmetric_limit(*(data[column] for column, _, _ in left))
    right_limit = symmetric_limit(data[column])
    ax_left.set_ylim(-left_limit, left_limit)
    ax_right.set_ylim(-right_limit, right_limit)
    ax_left.axhline(0.0, color="black", linewidth=0.6)

    # Mark the end of each target, labelled with the trial it ends
    for index, end in trial_ends(data):
        ax_left.axvline(end, color="grey", linestyle="--", linewidth=0.8)
        ax_left.annotate(
            f"T{index}",
            xy=(end, 1.0),
            xycoords=("data", "axes fraction"),
            xytext=(-2, -2),
            textcoords="offset points",
            ha="right",
            va="top",
            fontsize=8,
            color="grey",
        )

    ax_left.set_xlim(time[0], time[-1])
    ax_left.set_xlabel("Time (s)")
    ax_left.set_ylabel(left_label)
    ax_right.set_ylabel(right_label, color=right_colour)
    ax_right.tick_params(axis="y", which="both", colors=right_colour)

    # Major grid with a fainter minor grid between, five minor divisions per major on every axis
    for ax in (ax_left, ax_right):
        ax.xaxis.set_minor_locator(AutoMinorLocator(5))
        ax.yaxis.set_minor_locator(AutoMinorLocator(5))
    ax_left.grid(True, which="major", alpha=0.4)
    ax_left.grid(True, which="minor", alpha=0.15, linewidth=0.5)

    ax_left.legend(handles=lines, loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=len(lines))
    ax_left.set_title(title)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_error_over_time(data: dict[str, np.ndarray], out_dir: pathlib.Path) -> pathlib.Path:
    return plot_dual_axis(
        data,
        left=[("err_x", "err_x (m)", "tab:blue"), ("err_y", "err_y (m)", "tab:orange")],
        right=("err_yaw", "err_yaw (rad)", "tab:green"),
        left_label="Position error (m)",
        right_label="Heading error (rad)",
        title="Target − robot error per trial (dashed lines: end of target)",
        path=out_dir / "error_over_time.png",
    )


def plot_command_over_time(data: dict[str, np.ndarray], out_dir: pathlib.Path) -> pathlib.Path:
    return plot_dual_axis(
        data,
        left=[("cmd_vx", "cmd_vx (m/s)", "tab:blue"), ("cmd_vy", "cmd_vy (m/s)", "tab:orange")],
        right=("cmd_wz", "cmd_wz (rad/s)", "tab:green"),
        left_label="Commanded linear velocity (m/s)",
        right_label="Commanded angular velocity (rad/s)",
        title="Commanded walk velocity per trial (dashed lines: end of target)",
        path=out_dir / "command_over_time.png",
    )


FIGURES = [plot_error_over_time, plot_command_over_time]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", type=pathlib.Path, help="WalkPathBenchmark trajectory CSV")
    args = parser.parse_args()

    if not args.csv.is_file():
        print(f"No such file: {args.csv}", file=sys.stderr)
        return 1

    data = load(args.csv)
    out_dir = args.csv.parent / args.csv.stem
    out_dir.mkdir(parents=True, exist_ok=True)

    for figure in FIGURES:
        print(f"Wrote {figure(data, out_dir)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
