#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2021 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import os
import re
import shutil
import subprocess
import sys
from subprocess import DEVNULL

from termcolor import cprint

import b
from utility.shell import WrapPty

from . import defaults, platform


def _is_docker():
    path = "/proc/self/cgroup"
    return os.path.exists("/.dockerenv") or os.path.isfile(path) and any("docker" in line for line in open(path))


def _is_wsl1():
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


def _is_linux():
    kernel_release = subprocess.check_output(["uname", "-a"]).decode("utf-8").strip()
    is_macos_or_wsl = re.search("Darwin|Microsoft", kernel_release, re.IGNORECASE)
    return not is_macos_or_wsl


def _setup_volume(volume_name, clean_volume):
    volume = (
        volume_name
        if subprocess.call(["docker", "volume", "inspect", volume_name], stderr=DEVNULL, stdout=DEVNULL) == 0
        else None
    )

    # If we are cleaning, remove this volume so we can recreate it
    if volume is not None and clean_volume:
        if subprocess.call(["docker", "volume", "rm", volume]) != 0:
            raise RuntimeError("Docker volume rm returned a non-zero exit")
        volume = None

    # If we don't have a volume, make one
    if volume is None:
        if subprocess.call(["docker", "volume", "create", volume_name], stderr=DEVNULL, stdout=DEVNULL) == 0:
            volume = volume_name
        else:
            raise RuntimeError("Docker volume create returned a non-zero exit code")

    return volume


def _setup_internal_image(image, rebuild, clean_volume):
    # Extract the repository and platform
    repository, target = image.split(":")
    _, repository = repository.split("/")

    # Rebuild the image if we are rebuilding
    if rebuild:
        platform.build(repository, target)

    # Find the current target for this platform
    target = target if target != "selected" else platform.selected(repository, defaults.local_user)

    # Ensure the build volume exists and clean it if necessary
    build_volume_name = f"{repository}_{target}_{defaults.local_user}_build"
    build_volume = _setup_volume(build_volume_name, clean_volume)

    # Ensure the NUsight node_modules volume exists and clean it if necessary
    nusight_volume_name = f"{repository}_{defaults.local_user}_node_modules"
    nusight_volume = _setup_volume(nusight_volume_name, clean_volume)

    mounts = [
        "--mount",
        f"type=volume,source={build_volume},target=/home/{defaults.image_user}/build,consistency=delegated",
        "--mount",
        f"type=volume,source={nusight_volume},target=/home/{defaults.image_user}/{defaults.directory}/nusight2/node_modules,consistency=delegated",
    ]

    # Get the path to the users ssh configuration folder
    ssh_source = os.path.join(os.path.expanduser("~"), ".ssh")
    if os.path.exists(ssh_source):
        mounts.extend(
            [
                "--mount",
                f"type=bind,source={ssh_source},target=/home/{defaults.image_user}/.ssh,readonly",
            ]
        )

    return mounts


def run(func, image, hostname="docker", ports=[], docker_context=None):
    requested_image = image

    def _run(**kwargs):
        pty = WrapPty()

        # If we are running in docker, then execute the command as normal
        if _is_docker():
            func(**kwargs)
            exit(0)

        # If this is the run command, then use the binary name to determine if
        # the hostname should be docker or webots
        # Binaries containing 'webots' (ie in the webots folder) should be given the hostname 'webots'
        # to ensure the config files are chosen correctly
        docker_hostname = "webots"
        # if kwargs["command"] == "run":
        #     if any(["webots" in arg for arg in kwargs["args"]]):
        #         docker_hostname = "webots"

        # Docker arguments
        docker_args = [
            "docker",
            "container",
            "run",
            "--rm",
            "--tty",
            "--attach",
            "stdin",
            "--attach",
            "stdout",
            "--attach",
            "stderr",
            "--hostname",
            docker_hostname,
            "--interactive",
            "--env",
            f"EDITOR={os.environ.get('EDITOR', 'nano')}",
            "--privileged",
            "--group-add",
            "audio",
            "--group-add",
            "dialout",
            "--group-add",
            "video_host",
            "--group-add",
            "render_host",
            "--privileged",
        ]

        # Work out if we are using an internal image
        internal_image, image = defaults.internalise_image(requested_image)

        # Grab the extra volumes from the arguments
        for m in kwargs["mount"]:
            docker_args.extend(["--mount", m])
        for v in kwargs["volume"]:
            docker_args.extend(["--volume", v])

        # Pass through GPUs if requested
        if kwargs["gpus"] is not None:
            docker_args.extend(["--gpus", kwargs["gpus"]])

        # Pass through devices if requested
        for d in kwargs["device"]:
            docker_args.extend(["--device", d])

        # Check if we can find the image, and if not try to either build it or pull it
        rebuild = kwargs["rebuild"]
        image_found = subprocess.call(["docker", "image", "inspect", image], stderr=DEVNULL, stdout=DEVNULL) == 0

        # Perform the tasks that we only perform on the internal image and add on any extra arguments needed
        if internal_image:
            if rebuild:
                print(f"User requested rebuild for {image}, rebuilding from source")
            elif not image_found:
                print(f"Could not find the image {image}, rebuilding from source")

            docker_args.extend(
                _setup_internal_image(
                    image=image,
                    rebuild=rebuild or not image_found,
                    clean_volume=kwargs["clean"],
                )
            )

        # Either we were requested to rebuild, or there is no image so we had to rebuild
        elif rebuild or not image_found:
            if docker_context is not None:
                # Make it a relpath so it's easier to read
                context = os.path.relpath(docker_context, os.getcwd())
                if rebuild:
                    print(f"User requested rebuild of image {image}, rebuilding from {context}")
                elif not image_found:
                    print(f"Could not find the image {image}, rebuilding from {context}")
                if pty.spawn(["docker", "build", "-t", image, "."], cwd=context) != 0:
                    cprint(f"Failed to build {image} from {context}", "red", attrs=["bold"])
                    exit(1)

            # We are using a public image
            else:
                if rebuild:
                    print(f"User requested re-pull of latest {image} image")
                elif not image_found:
                    print(f"Could not find the image {image}, attempting to pull")

                if pty.spawn(["docker", "pull", image]) != 0:
                    cprint(f"The tag {image} failed to be pulled", "red", attrs=["bold"])
                    exit(1)

        # Work out what cwd we need to have on docker to mirror the cwd we have here
        code_to_cwd = os.path.relpath(os.getcwd(), b.project_dir)
        cwd_to_code = os.path.relpath(b.project_dir, os.getcwd())
        bind_path = b.project_dir

        # If we are running in WSL 1 we need to translate our path to a windows one for docker.
        # Docker with WSL 2 doesn't need this as it supports binding paths directly from WSL into a container.
        if _is_wsl1():
            bind_path = subprocess.check_output(["wslpath", "-m", b.project_dir])[:-1].decode("utf-8")

        # Mount the pwd in the docker image
        docker_args.extend(
            [
                "--mount",
                f"type=bind,source={bind_path},target=/home/{defaults.image_user}/{defaults.directory},consistency=cached",
                "--workdir",
                f"/home/{defaults.image_user}/{defaults.directory}/{code_to_cwd}",
            ]
        )

        # Set the user id to the current users id
        docker_args.extend(["--user", f"{os.getuid()}:{os.getgid()}"])

        # Network settings
        network = kwargs["network"]
        if network is None:
            network = "host" if _is_linux() else "bridge"

        docker_args.extend(["--network", network])

        # Environment variables
        # This should be specified as VAR1=VALUE1,VAR2=VALUE2
        if kwargs["environment"] is not None:
            key_pairs = [["--env", key_pair] for key_pair in kwargs["environment"].split(",")]
            for key_pair in key_pairs:
                docker_args.extend(key_pair)

        # Add the ports
        for port in ports:
            docker_args.extend(["--publish", port])

        # Choose the image
        docker_args.append(image)

        # Add the command
        docker_args.extend([f"{cwd_to_code}/b", *sys.argv[1:]])

        pty = WrapPty()
        exit(pty.spawn(docker_args))

    return _run
