#!/usr/bin/env python3

import os
import subprocess
import sys

import ruamel.yaml

BINARIES_DIR = "/home/nubots/NUbots/binaries"
CONFIG_DIR = BINARIES_DIR + "/config"

ENV_VARS = ["ROBOCUP_ROBOT_ID", "ROBOCUP_TEAM_COLOR", "ROBOCUP_SIMULATOR_ADDR"]


def read_args() -> dict:
    # Check that we have at least a role argument
    if len(sys.argv) < 2:
        print("Please specify a role to run!")
        sys.exit(1)

    role = sys.argv[1]
    is_goalie = False

    # Check for the goalie flag
    if len(sys.argv) > 2:
        is_goalie = sys.argv[2] == "--goalie"

    built_roles = []

    # Create a list of the built roles
    for filename in os.listdir(BINARIES_DIR):
        if os.path.isfile(os.path.join(BINARIES_DIR, filename)):
            built_roles.append(filename)

    # Ensure that the role requested is in the built roles
    if role not in built_roles:
        print("The role '" + role + "' does not exist!")
        sys.exit(1)

    # Read the env vars
    config = {var: os.environ.get(var) for var in ENV_VARS if var in os.environ}

    # Get the list of vars required that weren't set :(
    unset_vars = [var for var in ENV_VARS if var not in config]

    # Ensure that all required environment variables were set
    if len(unset_vars) != 0:
        print("The following environment variables were not set!")
        for var in unset_vars:
            print(var)
        sys.exit(1)

    return {"role": role, "env_vars": config, "is_goalie": is_goalie}


def set_env_vars(args: dict) -> None:
    # Get the data
    env_vars = args["env_vars"]
    is_goalie = args["is_goalie"]

    yaml = ruamel.yaml.YAML()

    # Change into the config directory
    os.chdir(CONFIG_DIR)

    # Set `player_id` in GlobalConfig.yaml from ROBOCUP_ROBOT_ID
    with open("GlobalConfig.yaml", "rw") as file:
        global_config = yaml.load(file)
        global_config["player_id"] = int(env_vars["ROBOCUP_ROBOT_ID"])
        yaml.dump(global_config, file)

    # Set `player_id` in GameController.yaml from ROBOCUP_ROBOT_ID
    with open("GameController.yaml", "rw") as file:
        game_controller_config = yaml.load(file)
        game_controller_config["player_id"] = int(env_vars["ROBOCUP_ROBOT_ID"])
        yaml.dump(game_controller_config, file)

    # ROBOCUP_TEAM_COLOR
    # ??

    # Set `server_address` and `port` in webots.yaml from ROBOCUP_SIMULATOR_ADDR
    with open("webots.yaml", "rw") as file:
        webots_config = yaml.load(file)
        address, port = env_vars["ROBOCUP_SIMULATOR_ADDR"].split(":", 2)
        webots_config["server_address"] = address
        webots_config["port"] = int(port)
        yaml.dump(webots_config, file)

    # Set `goalie` in SoccerSimulator.yaml from the --goalie argument
    with open("SoccerStrategy.yaml", "rw") as file:
        soccer_simulator_config = yaml.load(file)
        soccer_simulator_config["goalie"] = is_goalie
        yaml.dump(soccer_simulator_config, file)


def run_role(role: str) -> None:
    # Print the role we're running
    print(role)

    # Change into the directory with the binaries
    os.chdir(BINARIES_DIR)

    # Run the role binary
    exit(subprocess.run("./" + role).returncode)


if __name__ == "__main__":
    args = read_args()
    set_env_vars(args)
    run_role(args["role"])
