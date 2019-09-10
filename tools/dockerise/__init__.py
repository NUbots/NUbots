#!/usr/bin/env python3

import os
import b
import re
import sys

# Try to import docker, if we are already in docker this will fail
try:
    import docker
    import dockerpty

    client = docker.from_env()
except:
    client = None

repository = "nubots"


def is_docker():
    path = "/proc/self/cgroup"
    return os.path.exists("/.dockerenv") or os.path.isfile(path) and any("docker" in line for line in open(path))


def build_platform(platform):
    tag = "{}:{}".format(repository, platform)
    dockerdir = os.path.join(b.project_dir, "docker")

    # Build the image
    stream = client.api.build(
        path=dockerdir, tag=tag, buildargs={"platform": platform}, quiet=False, pull=True, rm=True, decode=True
    )

    # Print the progress as it happens
    for event in stream:
        if "stream" in event:
            sys.stdout.write(event["stream"])


def platforms():
    # Get the possible platforms
    p = re.compile("generate_([a-z0-9]+)_toolchain.py")
    return [
        m.group(1)
        for m in [p.match(s) for s in os.listdir(os.path.join(b.project_dir, "docker", "usr", "local", "toolchain"))]
        if m is not None
    ]


def get_selected_platform():
    try:
        selected = client.images.get("{}:selected".format(repository))
        names = [
            t.split(":")[-1]
            for t in selected.tags
            if t != "{}:selected".format(repository) and t.startswith("{}:".format(repository))
        ]
        if len(names) == 0:
            print("ERROR the currently selected platform is a dangling tag.")
            print("      The system is unable to work out what platform this was and will need to be reset")
            print("      run `./b target {platform}` to correct this")
            exit(1)
        elif len(names) == 1:
            return names[0]
        else:
            print("WARNING There are multiple platforms with the same image tag.")
            print("        The possible tags are [{}]".format(", ".join(names)))
            platform = list(sorted(names))[0]
            print("        The platform chosen will be {}".format(platform))
            return platform
    except docker.errors.ImageNotFound:
        print("No platform has been selected yet, defaulting to generic")
        return "generic"


def run_on_docker(func):
    if func.__name__ == "register":

        def register(command):

            # Add our docker specific options
            command.add_argument("--rebuild", action="store_true", help="rebuild the docker image checking for updates")
            command.add_argument(
                "--clean", action="store_true", help="delete and recreate all docker volumes (build directories)"
            )
            command.add_argument(
                "--platform",
                choices=platforms(),
                default="selected",
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
                # If the platform was "selected" that means to use the currently selected platform
                selected_platform = platform == "selected"
                if selected_platform:
                    platform = get_selected_platform()

                # Check if the image we want exists
                tag = "{}:{}".format(repository, platform)
                try:
                    client.images.get(tag)
                except docker.errors.ImageNotFound:
                    print("Could not find the image {}, rebuilding from source".format(tag))
                    rebuild = True

                # If we are requesting a rebuild, then run build
                if rebuild:
                    build_platform(platform)
                    # If we were building the selected platform we have to move our selected tag up
                    if selected_platform:
                        client.images.get(tag).tag(repository, "selected")

                # Find the volume for this platform
                build_volume_name = "{}_{}_build".format(repository, platform)
                try:
                    build_volume = client.volumes.get(build_volume_name)

                    # If we are cleaning, remove this volume so we can recreate it
                    if clean:
                        build_volume.remove(force=True)
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
                    working_dir="/home/{}/{}/{}".format(repository, "NUbots", code_to_cwd),
                    auto_remove=True,
                    tty=True,
                    stdin_open=True,
                    hostname="docker",
                    network_mode="host",
                    mounts=[
                        docker.types.Mount(
                            type="bind",
                            source=b.project_dir,
                            target="/home/{}/{}".format(repository, "NUbots"),
                            consistency="cached",
                        ),
                        docker.types.Mount(
                            type="volume",
                            source=build_volume.name,
                            target="/home/{}/build".format(repository),
                            consistency="delegated",
                        ),
                    ],
                )

                # Attach a pty to this terminal
                dockerpty.start(client.api, container.id)
                exit(container.wait()["StatusCode"])

        return run

    else:
        raise RuntimeError("run_on_docker must only be applied to the run and register functions")
