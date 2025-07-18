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
import shutil  # For file operations
import subprocess
import sys

from ruamel.yaml import YAML
from termcolor import cprint

import b
from utility.dockerise import run_on_docker

BUILD_DIR = "/home/nubots/build"
EXECUTABLE_PATH = os.path.join(BUILD_DIR, "bin/webots/localisation_benchmark")
LOCALISATION_YAML_PATH = os.path.join(BUILD_DIR, "config/webots/FieldLocalisationNLopt.yaml")
DESTINATION_DIR = os.path.join(BUILD_DIR, "recordings/optimisation_results")


def modify_yaml(params, localisation_yaml_path):
    """
    Modifies the YAML configuration files with the provided parameters while preserving comments and formatting.
    """
    yaml_parser = YAML()
    yaml_parser.indent(mapping=2, sequence=4, offset=2)

    # Update FieldLocalisationNLopt.yaml
    with open(localisation_yaml_path, "r") as file:
        localisation_config = yaml_parser.load(file)

    localisation_config["field_line_distance_weight"] = params["field_line_distance_weight"]
    localisation_config["field_line_intersection_weight"] = params["field_line_intersection_weight"]
    localisation_config["state_change_weight"] = params["state_change_weight"]
    localisation_config["goal_post_distance_weight"] = params["goal_post_distance_weight"]
    localisation_config["change_limit"] = [
        params["change_limit_x"],
        params["change_limit_y"],
        params["change_limit_theta"],
    ]
    localisation_config["max_association_distance"] = params["max_association_distance"]

    Q_values = [params["Q_x"], params["Q_y"], params["Q_theta"]]
    R_values = [params["R_x"], params["R_y"], params["R_theta"]]
    localisation_config["kalman"]["Q"] = [[Q_values[0], 0, 0], [0, Q_values[1], 0], [0, 0, Q_values[2]]]
    localisation_config["kalman"]["R"] = [[R_values[0], 0, 0], [0, R_values[1], 0], [0, 0, R_values[2]]]

    with open(localisation_yaml_path, "w") as file:
        yaml_parser.dump(localisation_config, file)


def parse_rmse_translation(output):
    """
    Parses the RMSE translation error from the module's output.
    """
    match = re.search(r"translation rmse error:\s+([0-9.]+)", output)
    if match:
        return float(match.group(1))
    else:
        print("Could not parse RMSE translation error.")
        print(output)
        return None


def parse_rmse_rotation(output):
    """
    Parses the RMSE rotation error from the module's output.
    """
    match = re.search(r"rotation rmse error:\s+([0-9.]+)", output)
    if match:
        return float(match.group(1))
    else:
        print("Could not parse RMSE rotation error.")
        print(output)
        return None


def run_benchmark(nbs_file):
    """
    Runs the LocalisationBenchmark module and captures the RMSE.
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
            timeout=20,  # Adjust timeout if needed
        )
        output = result.stdout
        rmse_translation = parse_rmse_translation(output)
        rmse_rotation = parse_rmse_rotation(output)
        print("RMSE Translation:", rmse_translation)
        print("RMSE Rotation:", rmse_rotation)

        if rmse_translation is None or rmse_rotation is None:
            return None

        total_rmse = rmse_translation + rmse_rotation
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
    The objective function for Optuna that defines the optimisation problem.
    """
    field_line_distance_weight = trial.suggest_float("field_line_distance_weight", 1e-2, 1e2, log=True)
    field_line_intersection_weight = trial.suggest_float("field_line_intersection_weight", 1e-2, 1e2, log=True)
    state_change_weight = trial.suggest_float("state_change_weight", 1e-2, 1e2, log=True)
    goal_post_distance_weight = trial.suggest_float("goal_post_distance_weight", 1e-2, 1e2, log=True)
    change_limit_x = trial.suggest_float("change_limit_x", 1e-2, 1.0)
    change_limit_y = trial.suggest_float("change_limit_y", 1e-2, 1.0)
    change_limit_theta = trial.suggest_float("change_limit_theta", 1e-2, 1.0)
    max_association_distance = trial.suggest_float("max_association_distance", 1e-2, 10.0)
    Q_x = trial.suggest_float("Q_x", 1e-6, 1e-2, log=True)
    Q_y = trial.suggest_float("Q_y", 1e-6, 1e-2, log=True)
    Q_theta = trial.suggest_float("Q_theta", 1e-6, 1e-2, log=True)
    R_x = trial.suggest_float("R_x", 1e-2, 1e1)
    R_y = trial.suggest_float("R_y", 1e-2, 1e1)
    R_theta = trial.suggest_float("R_theta", 1e-2, 1e1)

    params = {
        "field_line_distance_weight": field_line_distance_weight,
        "field_line_intersection_weight": field_line_intersection_weight,
        "state_change_weight": state_change_weight,
        "goal_post_distance_weight": goal_post_distance_weight,
        "change_limit_x": change_limit_x,
        "change_limit_y": change_limit_y,
        "change_limit_theta": change_limit_theta,
        "max_association_distance": max_association_distance,
        "Q_x": Q_x,
        "Q_y": Q_y,
        "Q_theta": Q_theta,
        "R_x": R_x,
        "R_y": R_y,
        "R_theta": R_theta,
    }

    modify_yaml(params, LOCALISATION_YAML_PATH)

    # Run the benchmark with user attribute nbs_file
    rmse = run_benchmark(trial.study.user_attrs["nbs_file"])

    if rmse is None:
        return float("inf")

    trial.set_user_attr("rmse_translation", parse_rmse_translation)
    trial.set_user_attr("rmse_rotation", parse_rmse_rotation)

    return rmse


@run_on_docker
def register(parser):
    """
    Register command-line arguments for the optimization tool.
    """
    parser.description = "Optimises YAML configuration parameters for the LocalisationBenchmark using Optuna."
    parser.add_argument("--nbs_file", type=str, required=True, help="Path to the .nbs file with ground truth data.")
    parser.add_argument("--n_trials", type=int, default=1000, help="Number of optimisation trials to run.")
    parser.add_argument("--hostname", type=str, default="webots", help="Specify the Docker hostname (default: webots).")


@run_on_docker(hostname="webots")
def run(n_trials, nbs_file, **kwargs):
    """
    Run the optimisation tool.
    """
    study = optuna.create_study(direction="minimize", study_name="LocalisationBenchmarkOptimisation")
    study.set_user_attr("nbs_file", nbs_file)

    # Enable progress bar
    study.optimize(objective, n_trials=n_trials, show_progress_bar=True)

    # Colored output for results
    cprint("\nBest parameters found:", "green", attrs=["bold"])
    for key, value in study.best_params.items():
        cprint(f"{key}: ", "cyan", end="")
        cprint(f"{value}", "yellow")

    cprint(f"\nBest RMSE: {study.best_value}", "green", attrs=["bold"])

    # Save configs
    modify_yaml(study.best_params, LOCALISATION_YAML_PATH)
    os.makedirs(DESTINATION_DIR, exist_ok=True)
    shutil.copy(LOCALISATION_YAML_PATH, os.path.join(DESTINATION_DIR, "BestFieldLocalisationNLopt.yaml"))
    cprint(f"\n✅ Best YAML files saved to {DESTINATION_DIR}", "green")

    # Visualisations
    fig_history = optuna.visualization.plot_optimization_history(study)
    fig_history.write_html(os.path.join(DESTINATION_DIR, "optimisation_history.html"))

    cprint("\n📈 Visualisation plots saved:", "blue")
    cprint(f" - optimisation_history.html", "cyan")
    cprint(f" - param_importances.html", "cyan")
    cprint(f" - parallel_coordinate.html", "cyan")
    cprint(f" - contour_plot.html", "cyan")
    cprint(f" - slice_plot.html\n", "cyan")
