#!/usr/bin/env python3

from . import defaults


def register(func, image, **kwargs):
    def _register(command):

        # Only if we are using the internal image does it make sense to provide things like rebuild and clean
        if defaults.internalise_image(image)[0]:
            command.add_argument(
                "--clean", action="store_true", help="delete and recreate all docker volumes (build directories)"
            )

        command.add_argument("--rebuild", action="store_true", help="rebuild the docker image checking for updates")
        command.add_argument(
            "--environment",
            dest="environment",
            help="Run the container with extra environment variables. Set variables as VAR1=VALUE1,VAR2=VALUE2",
        )
        command.add_argument("--network", dest="network", help="Run the container on the specified docker network")
        command.add_argument("--mount", default=[], action="append", help="--mount commands to pass to docker run")
        command.add_argument("--volume", default=[], action="append", help="--volume commands to pass to docker run")
        command.add_argument("--gpus", help="--gpus commands to pass to docker run")

        func(command)

    return _register
