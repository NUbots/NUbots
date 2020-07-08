#!/usr/bin/env python3

import os
import re
import shutil
import stat
import subprocess
import sys

import b

from .wrappty import WrapPty

# Try to import docker, if we are already in docker this will fail
try:
    import docker
    import dockerpty

    client = docker.from_env()
except:
    client = None

repository = "nubots"
user = "nubots"
directory = "NUbots"


def is_docker():
    path = "/proc/self/cgroup"
    return os.path.exists("/.dockerenv") or os.path.isfile(path) and any("docker" in line for line in open(path))


def build_platform(platform):
    from tqdm import tqdm

    tag = "{}:{}".format(repository, platform)
    dockerdir = os.path.join(b.project_dir, "docker")

    # Go through all the files and try to ensure that their permissions are correct
    # Otherwise caching will not work properly
    for dir_name, subdirs, files in os.walk(dockerdir):
        for f in files:
            p = os.path.join(dir_name, f)
            current = stat.S_IMODE(os.lstat(p).st_mode)
            os.chmod(p, current & ~(stat.S_IWGRP | stat.S_IWOTH))

    # Determine how many progress bars we can make
    num_bars = int(os.environ.get("LINES", 0))
    if num_bars == 0:
        num_bars = int(subprocess.check_output(["tput", "lines"])[:-1].decode("utf-8"))

    # Make the progress bars
    bars = {
        pos: {
            "bar": tqdm(unit="B", unit_scale=True, dynamic_ncols=True, leave=True, position=pos, desc="Pending ..."),
            "available": True,
            "id": None,
        }
        for pos in range(num_bars)
    }

    # Pull the latest version from dockerhub
    progress = {}
    queue = {}
    for event in client.api.pull(repository="nubots/nubots", tag=platform, stream=True, decode=True):
        try:
            # Check for available bars and move items from the queue to progress bars
            for pos in range(num_bars):
                if bars[pos]["available"]:
                    for id, data in queue.items():
                        bars[pos]["available"] = False
                        bars[pos]["id"] = id
                        bars[pos]["bar"].total = data["total"]
                        bars[pos]["bar"].n = data["current"]
                        bars[pos]["bar"].set_description("{} - {}".format(id, data["status"]))
                        progress[id] = {"bar": bars[pos]["bar"]}

                        del queue[id]
                        break

            # Get id and status from the current event
            id = int(event["id"], 16)
            status = event["status"]

            # If this is the first time we have seen this id, assign a progress bar if one is available otherwise add
            # it to the queue
            if id not in progress and id not in queue:
                # Check for an available progress bar
                bar = None
                for pos in range(num_bars):
                    if bars[pos]["available"]:
                        bars[pos]["available"] = False
                        bars[pos]["id"] = id
                        bar = bars[pos]["bar"]
                        break

                if bar is not None:
                    progress[id] = {"bar": bar}
                else:
                    # No available progress bars, add to queue
                    queue[id] = {"current": 0, "total": 0, "status": status}

            # If we have a value in progressDetail
            if (
                "progressDetail" in event
                and "current" in event["progressDetail"]
                and "total" in event["progressDetail"]
            ):
                current = int(event["progressDetail"]["current"])
                total = int(event["progressDetail"]["total"])
            else:
                current = None
                total = None

            if id in progress:
                # Update the status
                progress[id]["bar"].set_description("{} - {}".format(id, status))

                # Update bar values
                if current is not None and total is not None:
                    progress[id]["bar"].total = total
                    progress[id]["bar"].n = current

                # Complete statuses need to finish off the bar
                if "complete" in status or "exists" in status:
                    # Remove from progress
                    del progress[id]

                    # Free the progress bar
                    for pos in range(num_bars):
                        if bars[pos]["id"] == id:
                            bars[pos]["bar"].set_description("Pending ...")
                            bars[pos]["bar"].reset()
                            bars[pos]["id"] = None
                            bars[pos]["available"] = True

            else:
                # Find our position in the queue
                if current is not None and total is not None:
                    queue[id]["current"] = current
                    queue[id]["total"] = total
                queue[id]["status"] = status

        except KeyError:
            print(event["status"])
        except ValueError:
            print(event["status"])

    # Clean up all the bars
    for k, v in bars.items():
        v["bar"].reset()
        v["bar"].close()

    # Build the image
    for event in client.api.build(
        path=dockerdir, tag=tag, buildargs={"platform": platform}, quiet=False, pull=True, rm=True, decode=True,
    ):
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

                # If we are running in WSL we need to translate our path to a windows one for docker
                bind_path = (
                    b.project_dir
                    if shutil.which("wslpath") is None
                    else subprocess.check_output(["wslpath", "-m", b.project_dir])[:-1].decode("utf-8")
                )

                container = client.containers.create(
                    tag,
                    command=["{}/b".format(cwd_to_code), *sys.argv[1:]],
                    working_dir="/home/{}/{}/{}".format(user, directory, code_to_cwd),
                    auto_remove=True,
                    tty=True,
                    stdin_open=True,
                    hostname="docker",
                    network_mode="host",
                    mounts=[
                        docker.types.Mount(
                            type="bind",
                            source=bind_path,
                            target="/home/{}/{}".format(user, directory),
                            consistency="cached",
                        ),
                        docker.types.Mount(
                            type="volume",
                            source=build_volume.name,
                            target="/home/{}/build".format(user),
                            consistency="delegated",
                        ),
                    ],
                    environment={"EDITOR": os.environ.get("EDITOR", "nano")},
                )

                # Attach a pty to this terminal
                dockerpty.start(client.api, container.id)
                exit(container.wait()["StatusCode"])

        return run

    else:
        raise RuntimeError("run_on_docker must only be applied to the run and register functions")
