#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2025 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import os
import re
import shutil
import subprocess
import sys

from ruamel.yaml import YAML
from termcolor import cprint

import b
from utility.dockerise import run_on_docker

BUILD_DIR = "/home/nubots/build"
EXECUTABLE_PATH = os.path.join(BUILD_DIR, "bin/webots/odometry_benchmark")
SENSOR_FILTER_YAML_PATH = os.path.join(BUILD_DIR, "config/webots/SensorFilter.yaml")
DESTINATION_DIR = os.path.join(BUILD_DIR, "recordings/optimisation_results")


def modify_yaml(params, sensor_filter_yaml_path):
    """
    Modifies the SensorFilter YAML configuration file with the provided parameters
    while preserving comments and formatting.
    """
    yaml_parser = YAML()
    yaml_parser.indent(mapping=2, sequence=4, offset=2)

    # Update SensorFilter.yaml
    with open(sensor_filter_yaml_path, "r") as file:
        sensor_filter_config = yaml_parser.load(file)

    # Modify Kp and Ki under mahony
    sensor_filter_config["mahony"]["Kp"] = params["Kp"]
    sensor_filter_config["mahony"]["Ki"] = params["Ki"]

    with open(sensor_filter_yaml_path, "w") as file:
        yaml_parser.dump(sensor_filter_config, file)


def parse_rmse_translation(output):
    """
    Parses the Odometry RMSE translation error from the module's output.
    """
    match = re.search(r"Odometry translation RMSE error:\s+([0-9.]+)", output)
    if match:
        return float(match.group(1))
    else:
        print("Could not parse Odometry RMSE translation error.")
        print(output)
        return None


def parse_rmse_rotation(output):
    """
    Parses the Odometry RMSE rotation error from the module's output.
    """
    match = re.search(r"Odometry rotation RMSE error:\s+([0-9.]+)", output)
    if match:
        return float(match.group(1))
    else:
        print("Could not parse Odometry RMSE rotation error.")
        print(output)
        return None


def run_benchmark(nbs_file):
    """
    Runs the OdometryBenchmark module and captures the RMSE.
    """
    if not os.path.isfile(EXECUTABLE_PATH):
        print(f"Executable not found at {EXECUTABLE_PATH}")
        return None
    if not os.path.isfile(nbs_file):
        print(f"NBS file not found at {nbs_file}")
        return None

    command = [EXECUTABLE_PATH, nbs_file]

    try:
        result = subprocess.run(
            command,
            cwd=BUILD_DIR,
            capture_output=True,
            text=True,
            check=True,
            bufsize=1,
            timeout=20,
        )
        output = result.stdout
        rmse_translation = parse_rmse_translation(output)
        rmse_rotation = parse_rmse_rotation(output)
        print("Odometry RMSE Translation:", rmse_translation)
        print("Odometry RMSE Rotation (degrees):", rmse_rotation)

        if rmse_translation is None or rmse_rotation is None:
            return None
        # Just use the rotation error for now TODO: Add translation error
        total_rmse = rmse_rotation
        return total_rmse

    except subprocess.TimeoutExpired:
        print("Benchmark timed out.")
        return None

    except subprocess.CalledProcessError as e:
        print("An error occurred while running the benchmark:")
        print(e.stderr)
        return None


def objective(trial):
    """
    The objective function for Optuna that defines the optimisation problem for odometry.
    """
    # Define search space for Kp and Ki based on SensorFilter.yaml defaults/expectations
    kp = trial.suggest_float("Kp", 1e-2, 1e1, log=True)
    ki = trial.suggest_float("Ki", 1e-2, 1e1, log=True)

    params = {
        "Kp": kp,
        "Ki": ki,
    }

    modify_yaml(params, SENSOR_FILTER_YAML_PATH)

    # Run the benchmark with user attribute nbs_file
    rmse = run_benchmark(trial.study.user_attrs["nbs_file"])

    if rmse is None:
        return float("inf")  # Indicate failure
    return rmse


@run_on_docker
def register(parser):
    """
    Register command-line arguments for the optimization tool.
    """
    parser.description = (
        "Optimises Mahony filter gains (Kp, Ki) in SensorFilter.yaml using Optuna and OdometryBenchmark."
    )
    parser.add_argument("--nbs_file", type=str, required=True, help="Path to the .nbs file with ground truth data.")
    parser.add_argument("--n_trials", type=int, default=100, help="Number of optimisation trials to run.")
    parser.add_argument("--hostname", type=str, default="webots", help="Specify the Docker hostname (default: webots).")


@run_on_docker(hostname="webots")
def run(n_trials, nbs_file, **kwargs):
    """
    Run the optimisation tool.
    """
    study = optuna.create_study(direction="minimize", study_name="OdometryBenchmarkOptimisation")
    study.set_user_attr("nbs_file", nbs_file)

    # Enable progress bar
    study.optimize(objective, n_trials=n_trials, show_progress_bar=True)
    cprint("\nBest parameters found:", "green", attrs=["bold"])
    for key, value in study.best_params.items():
        cprint(f"{key}: ", "cyan", end="")
        cprint(f"{value}", "yellow")

    cprint(f"\nBest combined RMSE (Translation_m + Rotation_deg): {study.best_value}", "green", attrs=["bold"])

    # Save best config
    modify_yaml(study.best_params, SENSOR_FILTER_YAML_PATH)
    os.makedirs(DESTINATION_DIR, exist_ok=True)
    shutil.copy(SENSOR_FILTER_YAML_PATH, os.path.join(DESTINATION_DIR, "BestSensorFilter.yaml"))
    cprint(f"\n✅ Best SensorFilter YAML saved to {DESTINATION_DIR}/BestSensorFilter.yaml", "green")

    # Visualisations
    try:
        fig_history = optuna.visualization.plot_optimization_history(study)
        fig_history.write_html(os.path.join(DESTINATION_DIR, "odometry_optimisation_history.html"))
        fig_importance = optuna.visualization.plot_param_importances(study)
        fig_importance.write_html(os.path.join(DESTINATION_DIR, "odometry_param_importances.html"))
        cprint("\n📈 Visualisation plots saved:", "blue")
        cprint(f" - odometry_optimisation_history.html", "cyan")
        cprint(f" - odometry_param_importances.html", "cyan")
        cprint(f" - odometry_parallel_coordinate.html", "cyan")
        cprint(f" - odometry_slice_plot.html\n", "cyan")

    except Exception as e:
        cprint(f"\nCould not generate visualisation plots: {e}", "yellow")
