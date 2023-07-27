#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys

import ruamel.yaml

DEFAULT_BINARIES_DIR = os.path.abspath(os.path.join(os.sep, "home", "nubots", "NUbots", "binaries"))
DEFAULT_CONFIG_DIR = os.path.join(DEFAULT_BINARIES_DIR, "config")

REQUIRED_ENV_VARS = ("ROBOCUP_ROBOT_ID", "ROBOCUP_TEAM_COLOR", "ROBOCUP_SIMULATOR_ADDR")
OPTIONAL_ENV_VARS = (
    "ROBOCUP_TEAM_ID",
    "ROBOCUP_TEAM_PLAYER1_IP",
    "ROBOCUP_TEAM_PLAYER2_IP",
    "ROBOCUP_TEAM_PLAYER3_IP",
    "ROBOCUP_TEAM_PLAYER4_IP",
)


yaml = ruamel.yaml.YAML()  # Can't use `typ="safe"` since we want round-tripping to preserve comments


def read_config(file_path):
    try:
        with open(file_path, "r") as file:
            return yaml.load(file)
    except FileNotFoundError:
        print(f"config file not found: {file_path}")
        sys.exit(1)


def write_config(file_path, config):
    try:
        with open(file_path, "w") as file:
            yaml.dump(config, file)
    except FileNotFoundError:
        print(f"config file not found: {file_path}")
        sys.exit(1)


def read_args() -> dict:
    parser = argparse.ArgumentParser(description="Run a role for webots RoboCup")

    parser.add_argument("role", help="The role to run")
    parser.add_argument(
        "--binaries-dir", dest="binaries_dir", default=DEFAULT_BINARIES_DIR, help="The path to built role binaries"
    )
    parser.add_argument("--config-dir", dest="config_dir", default=DEFAULT_CONFIG_DIR, help="The path to config files")

    args = parser.parse_args()

    # Ensure that the role requested exists
    if not os.path.exists(os.path.join(args.binaries_dir, args.role)):
        print("The role '" + args.role + "' does not exist!")
        sys.exit(1)

    # Read the required environment variables
    env_vars = {var: os.environ.get(var) for var in REQUIRED_ENV_VARS if var in os.environ}

    # Get the list of required environment variables that weren't set :(
    unset_vars = [var for var in REQUIRED_ENV_VARS if var not in env_vars]

    # Ensure that all required environment variables were set
    if len(unset_vars) != 0:
        print("The following environment variables were not set!")
        for var in unset_vars:
            print(var)
        sys.exit(1)

    # Read the optional environment variables
    for var in OPTIONAL_ENV_VARS:
        if var in os.environ:
            env_vars[var] = os.environ.get(var)

    return {
        "role": args.role,
        "binaries_dir": args.binaries_dir,
        "config_dir": args.config_dir,
        "env_vars": env_vars,
    }


def update_config_files(args: dict) -> None:
    env_vars = args["env_vars"]

    # Change into the config directory
    os.chdir(args["config_dir"])

    # Set `player_id` in GlobalConfig.yaml from ROBOCUP_ROBOT_ID
    global_config = read_config("GlobalConfig.yaml")
    global_config["player_id"] = int(env_vars["ROBOCUP_ROBOT_ID"])
    write_config("GlobalConfig.yaml", global_config)

    # Set `player_id` in GameController.yaml from ROBOCUP_ROBOT_ID
    game_controller_config = read_config("GameController.yaml")
    game_controller_config["player_id"] = int(env_vars["ROBOCUP_ROBOT_ID"])
    write_config("GameController.yaml", game_controller_config)

    # ROBOCUP_TEAM_COLOR
    # (not set as it's not used by our code)

    # Set `server_address` and `port` in Webots.yaml from ROBOCUP_SIMULATOR_ADDR
    webots_config = read_config("Webots.yaml")
    address, port = env_vars["ROBOCUP_SIMULATOR_ADDR"].split(":", 2)
    webots_config["server_address"] = address
    webots_config["port"] = int(port)
    write_config("Webots.yaml", webots_config)

    # Set `team_id` if it is provided
    if "ROBOCUP_TEAM_ID" in env_vars:
        global_config = read_config("GlobalConfig.yaml")
        global_config["team_id"] = int(env_vars["ROBOCUP_TEAM_ID"])
        write_config("GlobalConfig.yaml", global_config)

    # Configure logging to /robocup-logs if it exists
    if os.path.exists("/robocup-logs"):
        # Compute the log directory for this robot, based on ROBOCUP_ROBOT_ID
        hostname = f"webots{env_vars['ROBOCUP_ROBOT_ID']}"
        robot_log_dir = f"/robocup-logs/{hostname}"

        # Ensure the log directory exists
        os.makedirs(robot_log_dir, exist_ok=True)

        # Configure FileLogHandler to write to the RoboCup log directory
        file_log_handler_config = read_config("FileLogHandler.yaml")
        file_log_handler_config["log_file"] = f"{robot_log_dir}/log"
        write_config("FileLogHandler.yaml", file_log_handler_config)

        # Configure DataLogging to write to the RoboCup log directory
        data_logging_config = read_config("webotsrobocup/DataLogging.yaml")
        data_logging_config["output"]["directory"] = robot_log_dir
        write_config("webotsrobocup/DataLogging.yaml", data_logging_config)


def run_role(role: str, binaries_dir: str, env_vars: dict) -> None:
    # Change into the directory with the binaries
    os.chdir(binaries_dir)

    modified_env = os.environ.copy()

    # Set up environment variables to pass through to the binary
    for key, value in env_vars.items():
        modified_env[key] = value

    # Set the robot hostname
    modified_env["ROBOT_HOSTNAME"] = f"webots{env_vars['ROBOCUP_ROBOT_ID']}"

    # Run the role binary
    while True:
        print(f"Binary '{role}' starting...")  # For debugging
        exit_code = subprocess.run(f"./{role}", env=modified_env).returncode
        print(f"Binary '{role}' crashed! Exit code: {exit_code}")  # For debugging


if __name__ == "__main__":
    args = read_args()
    print("main args: ", args)  # For Debugging
    update_config_files(args)
    run_role(args["role"], args["binaries_dir"], args["env_vars"])
