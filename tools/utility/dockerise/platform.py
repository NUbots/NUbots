#!/usr/bin/env python3
import json
import os
import re
import stat
import subprocess
from subprocess import DEVNULL

from termcolor import cprint

import b
from utility.shell import WrapPty

from . import defaults


def selected(repository):
    # Get information about the selected image
    try:
        img_info = json.loads(
            subprocess.Popen(
                ["docker", "image", "inspect", "{}:selected".format(repository)], stdout=subprocess.PIPE
            ).communicate()[0],
        )

        # Obtain only the tag of the image, so from nubots:generic we only obtain generic
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
            platform = sorted(names)[0]
            print("        The platform chosen will be {}".format(platform))
            return platform

    except subprocess.CalledProcessError:
        cprint("Docker image inspect call returned a non-zero exit code.", "blue", attrs=["bold"])
        cprint("We are assuming no platform has been selected yet, defaulting to generic", "blue", attrs=["bold"])
        return "generic"


def list():
    # Get the possible platforms
    p = re.compile("generate_([a-z0-9]+)_toolchain.py")
    return [
        m.group(1)
        for m in [p.match(s) for s in os.listdir(os.path.join(b.project_dir, "docker", "usr", "local", "toolchain"))]
        if m is not None
    ]


def build(repository, platform):
    pty = WrapPty()

    # If we are building the selected platform we need to work out what that refers to
    _selected = platform == "selected"
    platform = selected(repository) if _selected else platform

    remote_tag = "{0}/{0}:{1}".format(repository, platform)
    local_tag = "{0}:{1}".format(repository, platform)
    selected_tag = "{}:selected".format(repository)
    dockerdir = os.path.join(b.project_dir, "docker")

    # Go through all the files and try to ensure that their permissions are correct
    # Otherwise caching will not work properly
    for dir_name, subdirs, files in os.walk(dockerdir):
        for f in files:
            p = os.path.join(dir_name, f)
            current = stat.S_IMODE(os.lstat(p).st_mode)
            os.chmod(p, current & ~(stat.S_IWGRP | stat.S_IWOTH))

    # Create a buildx instance for building this image
    builder_name = "{}_{}".format(defaults.image, platform)
    if subprocess.run(["docker", "buildx", "inspect", builder_name], stderr=DEVNULL, stdout=DEVNULL).returncode != 0:
        subprocess.run(["docker", "buildx", "create", "--name", builder_name], stderr=DEVNULL, stdout=DEVNULL)
    subprocess.run(["docker", "buildx", "use", builder_name])

    # Get docker host IP from bridge network
    docker_gateway = (
        subprocess.check_output(
            ["docker", "network", "inspect", "bridge", "--format", "{{range .IPAM.Config}}{{.Gateway}}{{end}}"],
        )
        .strip()
        .decode("ascii")
    )

    # Build the image!
    err = pty.spawn(
        [
            "docker",
            "buildx",
            "build",
            "-t",
            local_tag,
            "--pull",
            "--cache-from=type=registry,ref={}/{}:{}".format(defaults.repository, defaults.image, platform),
            "--cache-to=type=inline,mode=max",
            "--build-arg",
            "platform={}".format(platform),
            "--build-arg",
            "user_uid={}".format(os.getuid()),
            "--output=type=docker",
            "--add-host=host.docker.internal:{}".format(docker_gateway),
            dockerdir,
        ]
    )
    if err != 0:
        cprint("Docker build returned exit code {}".format(err), "red", attrs=["bold"])
        exit(err)


def pull(repository, platform):
    # Define our tag strings
    remote_tag = "{0}/{0}:{1}".format(repository, platform)
    local_tag = "{0}:{1}".format(repository, platform)
    selected_tag = "{}:selected".format(repository)

    print("Pulling remote image", remote_tag)
    err = subprocess.run(["docker", "pull", remote_tag]).returncode
    if err != 0:
        raise RuntimeError("docker image pull returned a non-zero exit code")

    print("Tagging ", remote_tag, "as", local_tag)
    err = subprocess.run(["docker", "tag", remote_tag, local_tag]).returncode
    if err != 0:
        raise RuntimeError("docker image tag returned a non-zero exit code")
