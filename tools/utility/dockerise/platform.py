#!/usr/bin/env python3
import json
import os
import re
import stat
import subprocess

from termcolor import cprint

import b
from utility.shell import WrapPty


def selected(repository):
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
    dockerdir = os.path.join(b.project_dir, "docker")

    # Go through all the files and try to ensure that their permissions are correct
    # Otherwise caching will not work properly
    for dir_name, subdirs, files in os.walk(dockerdir):
        for f in files:
            p = os.path.join(dir_name, f)
            current = stat.S_IMODE(os.lstat(p).st_mode)
            os.chmod(p, current & ~(stat.S_IWGRP | stat.S_IWOTH))

    # Pull the latest version
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
            "platform={}".format(platform),
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

    # If we were building the selected platform then update our selected tag
    if _selected:
        err = subprocess.call(
            ["docker", "image", "tag", "{}:{}".format(repository, platform), "{}:selected".format(repository)],
            stdout=subprocess.DEVNULL,
        )
        if err != 0:
            raise RuntimeError("docker image tag returned a non-zero exit code")
