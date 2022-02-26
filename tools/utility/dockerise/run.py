#!/usr/bin/env python3

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


def _setup_internal_image(image, rebuild):

    # Extract the repository and platform
    repository, target = image.split(":")

    # Rebuild the image if we are rebuilding
    if rebuild:
        platform.build(repository, target)

    # Find the volume for this platform
    target = target if target != "selected" else platform.selected(repository)
    v_name = "{}_{}_build".format(repository, target)
    v = (
        v_name
        if subprocess.run(["docker", "volume", "inspect", v_name], stderr=DEVNULL, stdout=DEVNULL).returncode == 0
        else None
    )

    # If we don't have a volume, make one
    if v is None:
        if subprocess.run(["docker", "volume", "create", v_name], stderr=DEVNULL, stdout=DEVNULL).returncode == 0:
            v = v_name
        else:
            raise RuntimeError("Docker volume create returned a non-zero exit code")

    return ["--mount", "type=volume,source={},target=/home/{}/build,consistency=delegated".format(v, defaults.user)]


def run(func, image):
    def _run(**kwargs):
        pty = WrapPty()

        # If we are running in docker, then execute the command as normal
        if _is_docker():
            func(**kwargs)
            exit(0)

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
            "docker",
            "--interactive",
            "--env",
            "EDITOR={}".format(os.environ.get("EDITOR", "nano")),
            "--privileged",
        ]

        # Work out if we are using an internal image
        internal_image = defaults.is_internal_image(image)

        # Grab the extra volumes from the arguments
        for m in kwargs["mount"]:
            docker_args.extend(["--mount", m])
        for v in kwargs["volume"]:
            docker_args.extend(["--volume", v])

        # Pass through GPUs if requested
        if kwargs["gpus"] is not None:
            docker_args.extend(["--gpus", kwargs["gpus"]])

        # Check if we can find the image, and if not try to either build it or pull it
        rebuild = False
        if subprocess.run(["docker", "image", "inspect", image], stderr=DEVNULL, stdout=DEVNULL).returncode != 0:
            if internal_image:
                print("Could not find the image {}, rebuilding from source".format(image))
                rebuild = True
            else:
                print("Could not find the image {}, attempting to pull".format(image))
                if pty.spawn(["docker", "pull", image]) != 0:
                    cprint("The tag {} failed to be pulled".format(image), "red", attrs=["bold"])
                    exit(1)

        # Perform the tasks that we only perform on the internal image and add on any extra arguments needed
        if internal_image:
            docker_args.extend(_setup_internal_image(image=image, rebuild=(rebuild or kwargs["rebuild"])))

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
                "type=bind,source={},target=/home/{}/{},consistency=cached".format(
                    bind_path, defaults.user, defaults.directory
                ),
                "--workdir",
                "/home/{}/{}/{}".format(defaults.user, defaults.directory, code_to_cwd),
            ]
        )

        # Network settings
        if kwargs["network"] is None:
            docker_args.extend(["--network", "host"])
        else:
            docker_args.extend(["--network", kwargs["network"]])

        # Choose the image
        docker_args.append(image)

        # If we are cleaning then delete all files and directories in the build volume
        if kwargs["clean"]:
            exit_code = pty.spawn(
                docker_args + ["/bin/bash", "-c", "find /home/{}/build -mindepth 1 -delete".format(defaults.user)]
            )
            if exit_code != 0:
                cprint("Failed to clean the build volume", "red", attrs=["bold"])
                exit(exit_code)

        # Add the command
        docker_args.extend(["{}/b".format(cwd_to_code), *sys.argv[1:]])

        pty = WrapPty()
        exit(pty.spawn(docker_args))

    return _run
