#!/usr/bin/env python3

import os

from termcolor import cprint

import b
from utility.dockerise import run_on_docker
from utility.shell import WrapPty


@run_on_docker
def register(command):
    # Install help
    command.help = "Configure the project in a docker container"

    command.add_argument(
        "-i",
        "--interactive",
        dest="interactive",
        action="store_true",
        default=False,
        help="perform an interactive configuration using ccmake",
    )

    command.add_argument(
        "-r",
        "--enable-roles",
        action="store",
        dest="enabled_roles",
        default=None,
        help="Colon-separated list of roles to enable",
    )

    command.add_argument(
        "-R",
        "--disable-roles",
        action="store",
        dest="disabled_roles",
        default=None,
        help="Colon-separated list of roles to disable. If '-R' is set, '-r' is ignored",
    )

    command.add_argument("args", nargs="...", help="the arguments to pass through to cmake")


@run_on_docker
def run(interactive, enabled_roles, disabled_roles, args, **kwargs):
    pty = WrapPty()

    # Change into the build directory
    os.chdir(os.path.join(b.project_dir, "..", "build"))

    # If specific roles are disabled, then we enable all of the others
    if disabled_roles:
        # roles are a colon separated list, so we split on that character
        disabled_roles = disabled_roles.split(":")
        cprint("Disabling all roles except these: ", end=" ", color="red", attrs=["bold"])
        cprint(str(disabled_roles), color="green", attrs=["bold"])
        args += b.get_cmake_role_flags(disabled_roles, True)

    # If specific roles are enabled **and** specific roles aren't disabled,
    # then we enable only the enabled_roles
    elif enabled_roles:
        enabled_roles = enabled_roles.split(":")
        cprint("Enabling all roles except these:", end=" ", color="green", attrs=["bold"])
        cprint(str(enabled_roles), color="red", attrs=["bold"])
        args += b.get_cmake_role_flags(enabled_roles, False)

    # To pass arguments to the cmake command you put them after "--"
    # but "--"  isn't a valid argument for cmake, so we remove it here
    if "--" in args:
        args.remove("--")

    # If interactive then run ccmake else just run cmake
    if interactive:
        exit(
            pty.spawn(["ccmake", "-GNinja", "-DCMAKE_TOOLCHAIN_FILE=/usr/local/toolchain.cmake", *args, b.project_dir])
        )
    else:
        exit(pty.spawn(["cmake", "-GNinja", "-DCMAKE_TOOLCHAIN_FILE=/usr/local/toolchain.cmake", *args, b.project_dir]))
