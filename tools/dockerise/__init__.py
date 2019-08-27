#!/usr/bin/env python3

import os
import b
import sys

# Try to import docker, if we are already in docker this will fail
try:
    import docker
    import dockerpty

    client = docker.from_env()
except:
    client = None


def is_docker():
    path = "/proc/self/cgroup"
    return os.path.exists("/.dockerenv") or os.path.isfile(path) and any("docker" in line for line in open(path))


def run_on_docker(func):
    if func.__name__ == "register":

        def register(command):
            # Get the possible services
            services = os.listdir(os.path.join(b.project_dir, "docker", "usr", "local", "toolchain"))
            services = [s[:-3] for s in services if s.endswith(".sh")]

            # Add our docker specific options
            command.add_argument("--rebuild", action="store_true", help="rebuild the docker image checking for updates")
            command.add_argument(
                "--clean", action="store_true", help="delete and recreate all docker volumes (build directories)"
            )
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

        def run(rebuild, clean, platform, **kwargs):
            # If we are running in docker, then execute the command as normal
            if is_docker():
                func(rebuild=rebuild, clean=clean, platform=platform, **kwargs)
            # Otherwise go and re-run the b script in docker
            else:
                # Check if the image we want exists
                tag = "nubots:{}".format(platform)
                try:
                    img = client.images.get(tag)
                except docker.errors.ImageNotFound:
                    print("Could not find the image {}, rebuilding from source".format(tag))
                    rebuild = True

                # If we are requesting a rebuild, then run build
                if rebuild:
                    dockerdir = os.path.join(b.project_dir, "docker")
                    dockerfile = os.path.join(dockerdir, "nubots.Dockerfile")

                    # Build the image
                    stream = client.api.build(
                        path=dockerdir,
                        dockerfile=dockerfile,
                        tag=tag,
                        buildargs={"platform": platform},
                        quiet=False,
                        pull=True,
                        rm=True,
                        decode=True,
                    )

                    # Print the progress as it happens
                    for event in stream:
                        if "stream" in event:
                            sys.stdout.write(event["stream"])

                # Find the volume for this platform
                build_volume_name = "nubots_{}_build".format(platform)
                try:
                    build_volume = client.volumes.get(build_volume_name)

                    # If we are cleaning, remove this volume so we can recreate it
                    if clean:
                        build_volume.remove()
                        build_volume = None
                except docker.errors.NotFound:
                    build_volume = None

                # If we don't have a volume, make one
                if build_volume is None:
                    build_volume = client.volumes.create(build_volume_name)

                # Work out what cwd we need to have on docker to mirror the cwd we have here
                code_to_cwd = os.path.relpath(os.getcwd(), b.project_dir)
                cwd_to_code = os.path.relpath(b.project_dir, os.getcwd())

                container = client.containers.create(
                    tag,
                    command=["{}/b".format(cwd_to_code), *sys.argv[1:]],
                    working_dir="/home/nubots/NUbots/{}".format(code_to_cwd),
                    auto_remove=True,
                    tty=True,
                    stdin_open=True,
                    hostname="docker",
                    mounts=[
                        docker.types.Mount(
                            type="bind",
                            source=b.project_dir,
                            target="/home/nubots/NUbots",
                            read_only=True,
                            consistency="cached",
                        ),
                        docker.types.Mount(
                            type="volume",
                            source=build_volume.name,
                            target="/home/nubots/NUbots/build",
                            consistency="delegated",
                        ),
                    ],
                )

                # Attach a pty to this terminal
                dockerpty.start(client.api, container.id)

        return run

    else:
        raise RuntimeError("run_on_docker must only be applied to the run and register functions")
