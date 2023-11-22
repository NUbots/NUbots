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
