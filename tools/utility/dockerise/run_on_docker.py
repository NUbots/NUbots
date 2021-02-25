#!/usr/bin/env python3
import types

from . import defaults
from .register import register
from .run import run


def run_on_docker(_func=None, image=None, **kwargs):

    image = "{}:selected".format(defaults.image) if image is None else image

    # Someone just used the decorator with no arguments they want to run on our default image
    if isinstance(_func, types.FunctionType):
        func = _func

        # Decorate our function with the default arguments
        if func.__name__ == "register":
            return register(func, image)
        elif func.__name__ == "run":
            return run(func, image=image)
        else:
            raise RuntimeError("run_on_docker must only be applied to the run and register functions")

    # We are being given a more complex set of arguments so need to create our own docker setup
    elif image is not None:

        def _run_on_docker(func):
            # Decorate our function with the default arguments
            if func.__name__ == "register":
                return register(func, image, **kwargs)
            elif func.__name__ == "run":
                return run(func, image=image, **kwargs)
            else:
                raise RuntimeError("run_on_docker must only be applied to the run and register functions")

        return _run_on_docker

    else:
        raise RuntimeError("If you are providing custom options you must at least provide an image (using image=)")
