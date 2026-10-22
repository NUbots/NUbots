#!/usr/bin/env python3
"""Print the steady-state tracking error from a WalkPathBenchmark trajectory CSV, without plotting it.

Input: the per-tick CSV WalkPathBenchmark writes to its csv_directory, one row per tick:

    t,trial,x,y,yaw,z,target_x,target_y,target_yaw,err_x,err_y,err_yaw,cmd_vx,cmd_vy,cmd_wz

CSVs written before the err_* columns existed have them computed from the pose and target, with the yaw error wrapped
to [-pi, pi]. The yaw error is then unwrapped within each trial, so it can run past +-pi rather than jump by 2 pi.

It prints the same steady-state error as plot_walkbenchmark_error.py: each trial's error averaged over its last
--settle-time seconds (the window the benchmark ends a trial on, at the target or at rest), and its RMSE over the
trials for each axis, then the run's total time. The benchmark only ends a trial early once the robot has settled, so
a trial that ran to --trial-timeout hadn't, and is left out of the RMSE. Needs only numpy.

Usage:
    python3 tools/analysis/walkbench-rmse.py recordings/walk_path_benchmark/20261020T22_24_30.csv
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import numpy as np

# WalkPathBenchmark.yaml's settle_time and trial_timeout
SETTLE_TIME = 2.0
TRIAL_TIMEOUT = 40.0


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
    return columns


def unwrap_per_trial(angle: np.ndarray, trial: np.ndarray) -> np.ndarray:
    """Makes a wrapped angle continuous within each trial, so an error passing through +-pi (the robot facing away
    from the target heading) does not jump by 2 pi. Each trial is shifted by whole turns so it ends in [-pi, pi]:
    the error it settles to is the shortest one"""
    out = np.empty_like(angle)
    for index in np.unique(trial):
        rows = trial == index
        unwrapped = np.unwrap(angle[rows])
        out[rows] = unwrapped - 2.0 * np.pi * np.round(unwrapped[-1] / (2.0 * np.pi))
    return out


def steady_state_error(data: dict[str, np.ndarray], settle_time: float, trial_timeout: float) -> list[dict]:
    """Per trial: the error averaged over its last settle_time seconds, the robot's net speed then, and whether the
    trial settled (ended before trial_timeout, less a tick of slack)"""
    trials = []
    for index in np.unique(data["trial"]):
        rows = data["trial"] == index
        t = data["t"][rows]
        window = t >= t[-1] - settle_time
        tw = t[window]
        # Net planar speed over the window, from the ground-truth path. Net displacement rather than path length, as
        # the torso sways from step to step even while stepping on the spot
        xw, yw = data["x"][rows][window], data["y"][rows][window]
        speed = np.hypot(xw[-1] - xw[0], yw[-1] - yw[0]) / (tw[-1] - tw[0]) if tw[-1] > tw[0] else 0.0
        trials.append(
            {
                "trial": int(index),
                "err_x": float(np.mean(data["err_x"][rows][window])),
                "err_y": float(np.mean(data["err_y"][rows][window])),
                "err_yaw": float(np.mean(data["err_yaw"][rows][window])),
                "speed": float(speed),
                "settled": bool(t[-1] < trial_timeout - 0.2),
            }
        )
    return trials


def run_duration(data: dict[str, np.ndarray]) -> float:
    """Time over every trial of the run. `t` restarts at zero with each trial, so it is each trial's span summed"""
    return float(sum(np.ptp(data["t"][data["trial"] == index]) for index in np.unique(data["trial"])))


def print_steady_state(trials: list[dict], settle_time: float) -> None:
    print(f"Steady-state error, averaged over each trial's last {settle_time:g} s (target - robot)")
    print(
        f"{'trial':>5}  {'err_x (m)':>10}  {'err_y (m)':>10}  {'dist (m)':>9}  {'err_yaw (rad)':>13}  {'speed (m/s)':>11}"
    )
    for trial in trials:
        dist = np.hypot(trial["err_x"], trial["err_y"])
        note = "" if trial["settled"] else "  timed out, left out"
        print(
            f"{'T' + str(trial['trial']):>5}  {trial['err_x']:>+10.3f}  {trial['err_y']:>+10.3f}  {dist:>9.3f}  "
            f"{trial['err_yaw']:>+13.3f}  {trial['speed']:>11.3f}{note}"
        )

    settled = [trial for trial in trials if trial["settled"]]
    if not settled:
        print("\nNo trial settled, so there is no steady-state RMSE")
        return
    rmse = {
        axis: float(np.sqrt(np.mean([trial[axis] ** 2 for trial in settled]))) for axis in ("err_x", "err_y", "err_yaw")
    }
    rmse_dist = float(np.sqrt(np.mean([trial["err_x"] ** 2 + trial["err_y"] ** 2 for trial in settled])))
    print(f"\nSteady-state RMSE over {len(settled)} of {len(trials)} trials")
    print(f"  x        {rmse['err_x']:.3f} m")
    print(f"  y        {rmse['err_y']:.3f} m")
    print(f"  position {rmse_dist:.3f} m")
    print(f"  yaw      {rmse['err_yaw']:.3f} rad ({np.degrees(rmse['err_yaw']):.1f} deg)")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", type=pathlib.Path, help="WalkPathBenchmark trajectory CSV")
    parser.add_argument(
        "--settle-time",
        type=float,
        default=SETTLE_TIME,
        help="Seconds at the end of each trial the steady-state error is averaged over (default: %(default)s)",
    )
    parser.add_argument(
        "--trial-timeout",
        type=float,
        default=TRIAL_TIMEOUT,
        help="The benchmark's trial_timeout (s): a trial that ran this long never settled (default: %(default)s)",
    )
    args = parser.parse_args()

    if not args.csv.is_file():
        print(f"No such file: {args.csv}", file=sys.stderr)
        return 1

    data = load(args.csv)
    print_steady_state(steady_state_error(data, args.settle_time, args.trial_timeout), args.settle_time)
    print(f"\nTotal run time {run_duration(data):.1f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
