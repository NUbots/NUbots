#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys

import ruamel.yaml

DEFAULT_BINARIES_DIR = "/home/nubots/NUbots/binaries"
DEFAULT_CONFIG_DIR = DEFAULT_BINARIES_DIR + "/config"

REQUIRED_ENV_VARS = ("ROBOCUP_ROBOT_ID", "ROBOCUP_TEAM_COLOR", "ROBOCUP_SIMULATOR_ADDR")
OPTIONAL_ENV_VARS = (
    "ROBOCUP_TEAM_PLAYER1_IP",
    "ROBOCUP_TEAM_PLAYER2_IP",
    "ROBOCUP_TEAM_PLAYER3_IP",
    "ROBOCUP_TEAM_PLAYER4_IP",
)


def read_args() -> dict:
    parser = argparse.ArgumentParser(description="Run a role for webots RoboCup")

    parser.add_argument("role", help="The role to run")
    parser.add_argument("--goalie", action="store_true", default=False, help="Run the role with goalie behaviour")
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
        "is_goalie": args.goalie,
    }


def update_config(args: dict) -> None:
    env_vars = args["env_vars"]
    is_goalie = args["is_goalie"]

    yaml = ruamel.yaml.YAML()  # Can't use `typ="safe"` since we want round-tripping to preserve comments

    # Change into the config directory
    os.chdir(args["config_dir"])

    # Set `player_id` in GlobalConfig.yaml from ROBOCUP_ROBOT_ID
    with open("GlobalConfig.yaml", "r+") as file:
        global_config = yaml.load(file)
        global_config["player_id"] = int(env_vars["ROBOCUP_ROBOT_ID"])
        file.seek(0)
        yaml.dump(global_config, file)

    # Set `player_id` in GameController.yaml from ROBOCUP_ROBOT_ID
    with open("GameController.yaml", "r+") as file:
        game_controller_config = yaml.load(file)
        game_controller_config["player_id"] = int(env_vars["ROBOCUP_ROBOT_ID"])
        file.seek(0)
        yaml.dump(game_controller_config, file)

    # ROBOCUP_TEAM_COLOR
    # (not set as it's not used by our code)

    # Set `server_address` and `port` in webots.yaml from ROBOCUP_SIMULATOR_ADDR
    with open("webots.yaml", "r+") as file:
        webots_config = yaml.load(file)
        address, port = env_vars["ROBOCUP_SIMULATOR_ADDR"].split(":", 2)
        webots_config["server_address"] = address
        webots_config["port"] = int(port)
        file.seek(0)
        yaml.dump(webots_config, file)

    # Set `goalie` in SoccerSimulator.yaml from the --goalie argument
    if is_goalie:
        with open("SoccerStrategy.yaml", "r+") as file:
            soccer_simulator_config = yaml.load(file)
            soccer_simulator_config["goalie"] = is_goalie
            file.seek(0)
            yaml.dump(soccer_simulator_config, file)


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
    sys.exit(subprocess.run(f"./{role}", env=modified_env).returncode)


if __name__ == "__main__":
    args = read_args()
    update_config(args)
    run_role(args["role"], args["binaries_dir"], args["env_vars"])
