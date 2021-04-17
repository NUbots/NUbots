#!/usr/bin/env python3
from . import platform

repository = "nubots"
image = "nubots"
user = "nubots"
directory = "NUbots"


def is_internal_image(tag):
    split = tag.split(":")
    return len(split) == 2 and split[0] == image and (split[1] in platform.list() or split[1] == "selected")
