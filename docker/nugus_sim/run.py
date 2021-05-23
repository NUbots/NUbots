#!/usr/bin/env python3

import os
import subprocess
import sys
from typing import Tuple

import ruamel.yaml

BINARIES_DIR = "/home/nubots/NUbots/binaries"
CONFIG_DIR = BINARIES_DIR + "/config"

ENV_VARS = {
    "ROBOCUP_ROBOT_ID": "ROBOCUP_ROBOT_ID",
    "ROBOCUP_TEAM_COLOR": "ROBOCUP_TEAM_COLOR",
    "ROBOCUP_SIMULATOR_ADDR": "ROBOCUP_SIMULATOR_ADDR",
}


def read_args() -> Tuple[str, dict]:
    if len(sys.argv) != 2:
        print("Please specifiy a single role to run!")
        sys.exit(1)

    role = sys.argv[1]

    built_roles = []

    # Built roles
    for filename in os.listdir(BINARIES_DIR):
        if os.path.isfile(os.path.join(BINARIES_DIR, filename)):
            built_roles.append(filename)

    if role not in built_roles:
        print("The role '" + role + "' does not exist!")
        sys.exit(1)

    # Read env vars
    config = {k: os.environ.get(v) for k, v in ENV_VARS.items() if v in os.environ}

    # Dict of env vars that didn't get set :(
    unset_vars = {k: ENV_VARS[k] for k in ENV_VARS if k not in config}

    # If not all needed environment variables were set
    if len(unset_vars) != 0:
        print("The following environment variables were not set!")
        for var in unset_vars:
            print(var)
        sys.exit(1)

    return role, config


def set_env_vars(config: dict) -> None:
    yaml = ruamel.yaml.YAML()
    os.chdir(CONFIG_DIR)

    # ROBOCUP_ROBOT_ID
    global_config_fname = "GlobalConfig.yaml"
    with open(global_config_fname) as file:
        data = yaml.load(file)

    data["player_id"] = int(config["ROBOCUP_ROBOT_ID"])

    with open(global_config_fname, "w") as file:
        yaml.dump(data, file)

    game_controller_fname = "GameController.yaml"
    with open(game_controller_fname) as file:
        data = yaml.load(file)

    data["player_id"] = int(config["ROBOCUP_ROBOT_ID"])

    with open(game_controller_fname, "w") as file:
        yaml.dump(data, file)

    # ROBOCUP_TEAM_COLOR
    # ??

    # ROBOCUP_SIMULATOR_ADDR
    webots_config_fname = "webots.yaml"
    webots_addr, webots_port = config["ROBOCUP_SIMULATOR_ADDR"].split(":", 2)

    with open(webots_config_fname) as file:
        data = yaml.load(file)
    data["server_address"] = str(webots_addr)
    data["port"] = int(webots_port)

    with open(webots_config_fname, "w") as file:
        yaml.dump(data, file)


def run_role(inRole: str) -> None:
    print(inRole)
    # Change into binaries directory
    os.chdir(BINARIES_DIR)

    # Run Binary
    exit(subprocess.run("./" + inRole).returncode)


if __name__ == "__main__":
    role, env_vars = read_args()
    set_env_vars(env_vars)
    run_role(role)
