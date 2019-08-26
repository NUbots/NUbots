#!/usr/bin/env python3

import os
import shutil
import pty
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
            # In docker we just want to ignore the "docker" arguments since we pass them through
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
                    subprocess.check_output(
                        [
                            "docker-compose",
                            "--file={}".format(os.path.join(b.project_dir, "docker-compose.yml")),
                            "ps",
                            "--services",
                        ]
                    )
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
                func(rebuild=rebuild, platform=platform, **kwargs)
            # Otherwise go and re-run the b script in docker
            else:
                compose_file = os.path.join(b.project_dir, "docker-compose.yml")
                # If we are requesting a rebuild, then run build
                if rebuild:
                    pty.spawn(["docker-compose", "--file={}".format(compose_file), "build", platform])

                # Work out what cwd we need to have on docker to mirror the cwd we have here
                code_to_cwd = os.path.relpath(os.getcwd(), b.project_dir)
                cwd_to_code = os.path.relpath(b.project_dir, os.getcwd())
                pty.spawn(
                    [
                        "docker-compose",
                        "--file={}".format(compose_file),
                        "run",
                        "--rm",
                        "--workdir=/home/nubots/NUbots/{}".format(code_to_cwd),
                        platform,
                        "{}/b".format(cwd_to_code),
                        *sys.argv[1:],
                    ]
                )

        return run

    else:
        raise RuntimeError("run_on_docker must only be applied to the run and register functions")
