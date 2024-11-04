import os
import re
import subprocess
import sys

import optuna
import yaml


def modify_yaml(params, yaml_path):
    """
    Modifies the YAML configuration file with the provided parameters.
    """
    with open(yaml_path, 'r') as file:
        config = yaml.safe_load(file)

    # Update scalar parameters
    config['field_line_distance_weight'] = params['field_line_distance_weight']
    config['field_line_intersection_weight'] = params['field_line_intersection_weight']
    config['state_change_weight'] = params['state_change_weight']

    # Update matrix parameters for Kalman filter noise covariances
    Q_value = params['Q']
    R_value = params['R']
    config['kalman']['Q'] = [[Q_value, 0, 0], [0, Q_value, 0], [0, 0, Q_value]]
    config['kalman']['R'] = [[R_value, 0, 0], [0, R_value, 0], [0, 0, R_value]]

    with open(yaml_path, 'w') as file:
        yaml.safe_dump(config, file)

def parse_rmse_translation(output):
    """
    Parses the RMSE translation error from the module's output.
    """
    match = re.search(r'translation rmse error:\s+([0-9.]+)', output)
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
    match = re.search(r'rotation rmse error:\s+([0-9.]+)', output)
    if match:
        return float(match.group(1))
    else:
        print("Could not parse RMSE rotation error.")
        print(output)
        return None

def run_benchmark():
    """
    Runs the LocalisationBenchmark module and captures the RMSE.
    """
    # Paths to the executable and NBS file
    executable_path = '/home/nubots/build/bin/webots/localisation_benchmark'
    nbs_file = '/home/nubots/build/recordings/localisation/20241104T05_40_50.nbs'

    # Ensure the executable and NBS file exist
    if not os.path.isfile(executable_path):
        print(f"Executable not found at {executable_path}")
        return None
    if not os.path.isfile(nbs_file):
        print(f"NBS file not found at {nbs_file}")
        return None

    # Command to run
    command = [executable_path, nbs_file]

    try:
        # Set the working directory to /home/nubots/build
        result = subprocess.run(
            command,
            cwd='/home/nubots/build',
            capture_output=True,
            text=True,
            check=True,
            bufsize=1
        )
        output = result.stdout
        rmse_translation = parse_rmse_translation(output)
        rmse_rotation = parse_rmse_rotation(output)
        print(rmse_translation)
        print(rmse_rotation)

        if rmse_translation is None or rmse_rotation is None:
            return None

        # Combine the RMSE values (adjust weighting as needed)
        total_rmse = rmse_translation + rmse_rotation
        return total_rmse

    except subprocess.CalledProcessError as e:
        print("An error occurred while running the benchmark:")
        print(e.stderr)
        return None

def objective(trial):
    """
    The objective function for Optuna that defines the optimization problem.
    """
    # Suggest values for the parameters within the specified ranges
    field_line_distance_weight = trial.suggest_loguniform('field_line_distance_weight', 0.1, 100)
    field_line_intersection_weight = trial.suggest_loguniform('field_line_intersection_weight', 0.1, 100)
    state_change_weight = trial.suggest_loguniform('state_change_weight', 0.1, 10)
    Q_value = trial.suggest_loguniform('Q', 1e-6, 1e-2)
    R_value = trial.suggest_uniform('R', 0.1, 10)

    # Prepare the parameters dictionary
    params = {
        'field_line_distance_weight': field_line_distance_weight,
        'field_line_intersection_weight': field_line_intersection_weight,
        'state_change_weight': state_change_weight,
        'Q': Q_value,
        'R': R_value
    }

    # Modify the YAML file with the new parameters
    yaml_path = '/home/nubots/build/config/FieldLocalisationNLopt.yaml'
    modify_yaml(params, yaml_path)

    # Run the benchmark and get the RMSE
    rmse = run_benchmark()

    if rmse is None:
        # Return a high value to penalize the trial if RMSE could not be obtained
        return float('inf')

    # Optionally, report intermediate values
    trial.set_user_attr('rmse_translation', parse_rmse_translation)
    trial.set_user_attr('rmse_rotation', parse_rmse_rotation)

    # Return the RMSE as the objective value to minimize
    return rmse

if __name__ == '__main__':
    # Create an Optuna study object
    study = optuna.create_study(direction='minimize', study_name='LocalisationBenchmarkOptimization')

    # Start the optimization
    study.optimize(objective, n_trials=50)  # Adjust n_trials as needed

    # Print the best parameters and RMSE
    print('Best parameters found:')
    for key, value in study.best_params.items():
        print(f'{key}: {value}')
    print(f'Best RMSE: {study.best_value}')

    # # Paths to the executable and NBS file
    # executable_path = '/home/nubots/build/bin/webots/localisation_benchmark'
    # nbs_file = '/home/nubots/build/recordings/localisation/20241104T05_40_50.nbs'

    # # Ensure the executable and NBS file exist
    # if not os.path.isfile(executable_path):
    #     print(f"Executable not found at {executable_path}")
    # if not os.path.isfile(nbs_file):
    #     print(f"NBS file not found at {nbs_file}")

    # # Command to run
    # command = [executable_path, nbs_file]

    # result = subprocess.run(
    #     command,
    #     cwd='/home/nubots/build',
    #     capture_output=True,
    #     text=True,
    #     check=True,
    #     bufsize=1
    # )
    # output = result.stdout
    # print(output)
    # rmse_translation = parse_rmse_translation(output)
    # rmse_rotation = parse_rmse_rotation(output)
    # print(rmse_translation)
    # print(rmse_rotation)
