#!/usr/bin/env python3
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
