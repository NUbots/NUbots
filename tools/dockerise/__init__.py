#!/usr/bin/env python3

import json
import os
import re
import shutil
import stat
import subprocess
import sys

from termcolor import cprint

import b

from .wrappty import WrapPty

repository = "nubots"
user = "nubots"
directory = "NUbots"


def is_docker():
    path = "/proc/self/cgroup"
    return os.path.exists("/.dockerenv") or os.path.isfile(path) and any("docker" in line for line in open(path))


def is_wsl1():
    if shutil.which("wslpath") is None:
        return False

    kernel_release = subprocess.check_output(["uname", "-r"]).decode("utf-8").strip()
    search = re.search("^(\d+)\.(\d+)\.\d+", kernel_release)
    major = search.group(1)
    minor = search.group(2)

    if major is None or minor is None:
        return False

    # WSL 2 has kernel release version >= 4.19 (https://askubuntu.com/a/1177730)
    return int(major) <= 4 and int(minor) < 19


def build_platform(platform):
    pty = WrapPty()

    remote_tag = "{0}/{0}:{1}".format(repository, platform)
    local_tag = "{0}:{1}".format(repository, platform)
    dockerdir = os.path.join(b.project_dir, "docker")

    # Go through all the files and try to ensure that their permissions are correct
    # Otherwise caching will not work properly
    for dir_name, subdirs, files in os.walk(dockerdir):
        for f in files:
            p = os.path.join(dir_name, f)
            current = stat.S_IMODE(os.lstat(p).st_mode)
            os.chmod(p, current & ~(stat.S_IWGRP | stat.S_IWOTH))

    # Pull the latest version from dockerhub
    err = pty.spawn(["docker", "pull", remote_tag])
    if err != 0:
        cprint("Docker pull returned exit code {}".format(err), "red", attrs=["bold"])
        exit(err)

    build_env = os.environ
    build_env["DOCKER_BUILDKIT"] = "1"
    old_cwd = os.getcwd()
    os.chdir(dockerdir)
    err = pty.spawn(
        [
            "docker",
            "build",
            ".",
            "--build-arg",
            "BUILDKIT_INLINE_CACHE=1",
            "--build-arg",
            "platform={}".format(platform if platform != "buildkit" else "generic"),
            "--build-arg",
            "user_uid={}".format(os.getuid()),
            "-t",
            local_tag,
        ],
        env=build_env,
    )
    os.chdir(old_cwd)
    if err != 0:
        cprint("Docker build returned exit code {}".format(err), "red", attrs=["bold"])
        exit(err)


def platforms():
    # Get the possible platforms
    p = re.compile("generate_([a-z0-9]+)_toolchain.py")
    return [
        m.group(1)
        for m in [p.match(s) for s in os.listdir(os.path.join(b.project_dir, "docker", "usr", "local", "toolchain"))]
        if m is not None
    ]


def get_selected_platform():
    # Get information about the selected image
    try:
        img_info = json.loads(
            subprocess.Popen(
                ["docker", "image", "inspect", "{}:selected".format(repository)], stdout=subprocess.PIPE
            ).communicate()[0],
        )

        names = [
            tag.split(":")[-1]
            for tag in img_info[0]["RepoTags"]
            if tag != "{}:selected".format(repository) and tag.startswith("{}:".format(repository))
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

    except subprocess.CalledProcessError:
        cprint("Docker image inspect call returned a non-zero exit code.", "blue", attrs=["bold"])
        cprint("We are assuming no platform has been selected yet, defaulting to generic", "blue", attrs=["bold"])
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
            command.add_argument("--network", dest="network", help="Run the container on the specified docker network")

            func(command)

        return register

    elif func.__name__ == "run":

        def run(rebuild, clean, platform, network, **kwargs):
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
                if subprocess.call(["docker", "image", "inspect", tag], stdout=subprocess.DEVNULL) != 0:
                    print("Could not find the image {}, rebuilding from source".format(tag))
                    rebuild = True

                # If we are requesting a rebuild, then run build
                if rebuild:
                    build_platform(platform)
                    # If we were building the selected platform we have to move our selected tag up
                    if selected_platform:
                        if (
                            subprocess.call(
                                ["docker", "image", "tag", tag, "{}:selected".format(repository)],
                                stdout=subprocess.DEVNULL,
                            )
                            != 0
                        ):
                            cprint("docker image tag returned a non-zero exit code", "red", attrs=["bold"])
                            exit(1)

                # Find the volume for this platform
                build_volume_name = "{}_{}_build".format(repository, platform)
                if subprocess.call(["docker", "volume", "inspect", build_volume_name], stdout=subprocess.DEVNULL) == 0:
                    build_volume = build_volume_name
                else:
                    build_volume = None

                # If we are cleaning, remove this volume so we can recreate it
                if build_volume and clean:
                    if subprocess.call(["docker", "volume", "rm", build_volume_name], stdout=subprocess.DEVNULL) != 0:
                        cprint("Docker volume rm returned a non-zero exit", "red", attrs=["bold"])
                        exit(1)

                    build_volume = None

                # If we don't have a volume, make one
                if build_volume is None:
                    if (
                        subprocess.call(["docker", "volume", "create", build_volume_name], stdout=subprocess.DEVNULL)
                        == 0
                    ):
                        build_volume = build_volume_name
                    else:
                        cprint("Docker volume create returned a non-zero exit code", "red", attrs=["bold"])
                        exit(1)

                # Work out what cwd we need to have on docker to mirror the cwd we have here
                code_to_cwd = os.path.relpath(os.getcwd(), b.project_dir)
                cwd_to_code = os.path.relpath(b.project_dir, os.getcwd())

                bind_path = b.project_dir

                # If we are running in WSL 1 we need to translate our path to a windows one for docker.
                # Docker with WSL 2 doesn't need this as it supports binding paths directly from WSL into a container.
                if is_wsl1():
                    bind_path = subprocess.check_output(["wslpath", "-m", b.project_dir])[:-1].decode("utf-8")

                pty = WrapPty()
                exit(
                    pty.spawn(
                        [
                            "docker",
                            "container",
                            "run",
                            "--rm",
                            "--tty",
                            "--workdir",
                            "/home/{}/{}/{}".format(user, directory, code_to_cwd),
                            "--name",
                            "{}_{}".format(repository, platform),
                            "--attach",
                            "stdin",
                            "--attach",
                            "stdout",
                            "--attach",
                            "stderr",
                            "--hostname",
                            "docker",
                            "--network",
                            network or "host",
                            "--mount",
                            "type=bind,source={},target=/home/{}/{},consistency=cached".format(
                                bind_path, user, directory
                            ),
                            "--mount",
                            "type=volume,source={},target=/home/{}/build,consistency=delegated".format(
                                build_volume, user
                            ),
                            "--env",
                            "EDITOR={}".format(os.environ.get("EDITOR", "nano")),
                            "--interactive",
                            tag,
                            "{}/b".format(cwd_to_code),
                            *sys.argv[1:],
                        ]
                    )
                )

        return run

    else:
        raise RuntimeError("run_on_docker must only be applied to the run and register functions")
