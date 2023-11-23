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
import types

from . import defaults
from .register import register
from .run import run


def run_on_docker(_func=None, image=None, **kwargs):
    image = defaults.image_name("selected") if image is None else image

    # Someone just used the decorator with no arguments they want to run on our default image
    if isinstance(_func, types.FunctionType):
        func = _func

        # Decorate our function with the default arguments
        if func.__name__ == "register":
            return register(func, image=image)
        elif func.__name__ == "run":
            return run(func, image=image)
        else:
            raise RuntimeError("run_on_docker must only be applied to the run and register functions")

    def _run_on_docker(func):
        # Decorate our function with the default arguments
        if func.__name__ == "register":
            return register(func, image=image, **kwargs)
        elif func.__name__ == "run":
            return run(func, image=image, **kwargs)
        else:
            raise RuntimeError("run_on_docker must only be applied to the run and register functions")

    return _run_on_docker
