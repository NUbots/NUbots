#!/usr/bin/env python3
import getpass
import os

from . import platform

cache_registry = "nubots"
image = "nubots"
image_user = "nubots"
directory = "NUbots"

# This can fail if the local user id doesn't exist (e.g. you're in docker and set a uid)
# In those cases, we really don't care so we just use the local userid instead
try:
    local_user = getpass.getuser()
except:
    local_user = f"{os.getuid()}"


def image_name(label, image=image, username=local_user):
    return f"{username}/{image}:{label}"


def internalise_image(tag, username=local_user):
    # Check if this is an internal image
    split = tag.split(":")
    is_internal = (
        len(split) == 2
        and split[0] in [image, f"{username}/{image}"]
        and (split[1] in platform.list() or split[1] == "selected")
    )

    if is_internal:
        return True, f"{username}/{image}:{split[1]}"
    else:
        return False, tag
