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


def selected(image, username):
    # Get information about the selected image
    try:
        selected_img = defaults.image_name("selected", image, username)
        prefix = "{}:".format(selected_img.split(":")[0])
        img_info = json.loads(
            subprocess.Popen(["docker", "image", "inspect", selected_img], stdout=subprocess.PIPE).communicate()[0]
        )

        names = [tag.split(":")[1] for tag in img_info[0]["RepoTags"] if tag != selected_img and tag.startswith(prefix)]
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
            print(f"        The platform chosen will be {platform}")
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


def build(image, platform, username, uid, reset):
    pty = WrapPty()

    # If we are building the selected platform we need to work out what that refers to
    _selected = platform == "selected"
    platform = selected(image) if _selected else platform

    local_tag = defaults.image_name(platform, image, username)
    dockerdir = os.path.join(b.project_dir, "docker")

    # Go through all the files and try to ensure that their permissions are correct
    # Otherwise caching will not work properly
    for dir_name, dirs, files in os.walk(dockerdir):
        for path in files + dirs:
            try:
                p = os.path.join(dir_name, path)
                current = stat.S_IMODE(os.lstat(p).st_mode)
                os.chmod(p, current & ~(stat.S_IWGRP | stat.S_IWOTH))
            except:
                pass

    # Create a buildx instance for building images (for this user)
    builder_name = defaults.image

    # Check to see if the buildx instance has already been created
    buildx_exists = (
        subprocess.run(["docker", "buildx", "inspect", builder_name], stderr=DEVNULL, stdout=DEVNULL).returncode == 0
    )

    # Remove the buildx instance if it exists and a reset was requested
    if buildx_exists and reset:
        err = subprocess.run(["docker", "buildx", "rm", builder_name], stderr=DEVNULL, stdout=DEVNULL).returncode
        if err != 0:
            cprint(f"Docker buildx rm returned exit code {err}", "red", attrs=["bold"])
            exit(err)
        buildx_exists = False

    # Create the buildx instance if it doesn't exist
    if not buildx_exists:
        subprocess.run(
            [
                "docker",
                "buildx",
                "create",
                "--name",
                builder_name,
                # Disable clipping of the output logs
                "--driver-opt",
                "env.BUILDKIT_STEP_LOG_MAX_SIZE=-1",
                "--driver-opt",
                "env.BUILDKIT_STEP_LOG_MAX_SPEED=-1",
            ],
            stderr=DEVNULL,
            stdout=DEVNULL,
        )

    # Make sure buildx will use the correct buildx instance
    subprocess.run(["docker", "buildx", "use", builder_name])

    # Build the image!
    err = pty.spawn(
        [
            "docker",
            "buildx",
            "build",
            "-t",
            local_tag,
            "--pull",
            "--build-arg",
            f"platform={platform}",
            "--build-arg",
            f"user_uid={uid}",
            "--output=type=docker",
            f"--cache-from=type=registry,ref={defaults.cache_registry}/{image}:{platform}",
            f"--cache-to=type=inline",
            dockerdir,
        ]
    )
    if err != 0:
        cprint(f"Docker build returned exit code {err}", "red", attrs=["bold"])
        exit(err)

    # If we were building the selected platform then update our selected tag
    if _selected:
        err = subprocess.call(
            [
                "docker",
                "image",
                "tag",
                defaults.image_name(platform, image, username),
                defaults.image_name("selected", image, username),
            ],
            stdout=subprocess.DEVNULL,
        )
        if err != 0:
            raise RuntimeError("docker image tag returned a non-zero exit code")
