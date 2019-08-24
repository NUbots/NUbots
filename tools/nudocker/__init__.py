#!/usr/bin/env python3

import os
import shutil
import b
import subprocess
import sys


def is_docker():
    path = "/proc/self/cgroup"
    return os.path.exists("/.dockerenv") or os.path.isfile(path) and any("docker" in line for line in open(path))


def run_on_docker(func):
    if func.__name__ == "register":
        # We only add the docker options if we are not already in docker
        if is_docker():
            # In docker we just want to ignore the "docker" arguments
            def register(command):
                command.add_argument("--rebuild", action="store_true")
                command.add_argument("--platform", nargs="?")

            return register
        else:
            # Check that docker-compose binary exists as we need it to run docker
            if shutil.which("docker-compose") is None:
                raise RuntimeError("Docker compose is required to run docker based scripts")

            def register(command):
                # Get the possible services
                services = (
                    subprocess.Popen(
                        ["docker-compose", "ps", "--services"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT
                    )
                    .communicate()[0]
                    .decode("utf-8")
                    .split()
                )

                # Add our docker specific options
                command.add_argument("--rebuild", action="store_true", help="Build the docker container")
                command.add_argument(
                    "--platform",
                    choices=services,
                    default="generic",
                    nargs="?",
                    help="The image to use for the docker container",
                )

                func(command)

            return register

    elif func.__name__ == "run":

        def run(rebuild, platform, **kwargs):
            # If we are running in docker, then execute the command as normal
            if is_docker():
                print("I'm now running in docker!")
                func(**kwargs)
            # Otherwise go and re-run the b script in docker
            else:
                # If we are requesting a rebuild, then run build
                print("I'm not running in docker... yet!")
                p = subprocess.Popen(["docker-compose", "run", platform, "./b", *sys.argv[1:]])
                p.communicate()

        return run

    else:
        raise RuntimeError("run_on_docker must only be applied to the run and register functions")
