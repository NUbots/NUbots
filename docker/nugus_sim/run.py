#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys

import ruamel.yaml

BINARIES_DIR = "/home/nubots/NUbots/binaries"
CONFIG_DIR = BINARIES_DIR + "/config"

ENV_VARS = ("ROBOCUP_ROBOT_ID", "ROBOCUP_TEAM_COLOR", "ROBOCUP_SIMULATOR_ADDR")


def read_args() -> dict:
    parser = argparse.ArgumentParser(description="Run a role for webots RoboCup")

    parser.add_argument("role", help="The role to run")
    parser.add_argument(
        "--goalie", action=argparse.BooleanOptionalAction, default=False, help="Run the role with goalie behaviour"
    )

    args = parser.parse_args()

    # Ensure that the role requested exists
    if not os.path.exists(os.path.join(BINARIES_DIR, args.role)):
        print("The role '" + args.role + "' does not exist!")
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

    return {"role": args.role, "env_vars": config, "is_goalie": args.goalie}


def update_config(args: dict) -> None:
    # Get the data
    env_vars = args["env_vars"]
    is_goalie = args["is_goalie"]

    yaml = ruamel.yaml.YAML(typ="safe")

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
    # Change into the directory with the binaries
    os.chdir(BINARIES_DIR)

    # Run the role binary
    exit(subprocess.run(f"./{role}").returncode)


if __name__ == "__main__":
    args = read_args()
    update_config(args)
    run_role(args["role"])
