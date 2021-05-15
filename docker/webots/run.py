#!/usr/bin/env python3

import os
import subprocess
import sys

BINARIES_DIR = "/home/nubots/NUbots/binaries"


def main():
    if len(sys.argv) != 2:
        print("Please specifiy a single role to run!")
        sys.exit(1)

    run_role = sys.argv[1]

    built_roles = []

    # Built roles
    for filename in os.listdir(BINARIES_DIR):
        if os.path.isfile(os.path.join(BINARIES_DIR, filename)):
            built_roles.append(filename)

    if run_role not in built_roles:
        print("That role does not exist!")
        sys.exit(0)

    env_vars = {
        "ROBOCUP_ROBOT_ID": "ROBOCUP_ROBOT_ID",
        "ROBOCUP_TEAM_COLOR": "ROBOCUP_TEAM_COLOR",
        "ROBOCUP_SIMULATOR_ADDR": "ROBOCUP_SIMULATOR_ADDR",
    }

    # Read environment variables
    config = {k: os.environ.get(v) for k, v in env_vars.items() if v in os.environ}

    unset_vars = {k: env_vars[k] for k in env_vars if k not in config}

    # If not all needed environment variables were set
    if len(unset_vars) != 0:
        print("The following environment variables were not set!")
        for var in unset_vars:
            print(var)

    # Set environment variables
    setEnvVars(config)

    # Run Role!


def setEnvVars(config: dict):
    # print(config)
    # Set args in appropriate config files
    pass


def runRole(inRole: str):
    # Change into binaries directory
    os.chdir(BINARIES_DIR)

    # Run Binary
    err = subprocess.run([inRole]).returncode
    if err != 0:
        print("returned exit code {}".format(err))
        exit(err)


if __name__ == "__main__":
    main()
