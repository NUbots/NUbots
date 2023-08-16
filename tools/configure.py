#!/usr/bin/env python3

import argparse
import glob
import os

import b
from utility.dockerise import run_on_docker
from utility.roles import all_role_names, dir_role_names, role_folders
from utility.shell import WrapPty

ROLE_GROUPS = {"all": all_role_names()}

for r in role_folders():
    ROLE_GROUPS[r] = dir_role_names(r)


class ExtendConstAction(argparse._AppendConstAction):
    def __call__(self, parser, namespace, values, option_string=None):
        items = getattr(namespace, self.dest, None)
        items = argparse._copy_items(items)
        items.extend(self.const)
        setattr(namespace, self.dest, items)


@run_on_docker
def register(command):
    # Install help
    command.description = "Configure the project in a docker container"
    role_selection_args = command.add_argument_group(
        "role selection", "Select which roles to enable or disable via the command line (supports globbing)"
    )

    command.add_argument(
        "-i",
        "--interactive",
        dest="interactive",
        action="store_true",
        default=False,
        help="perform an interactive configuration using ccmake",
    )
    role_selection_args.add_argument(
        "--unset_roles",
        dest="unset_roles",
        action="extend",
        default=[],
        nargs="*",
        help="Disable roles matching provided UNSET_ROLES (supports globbing)",
    )
    for group in ROLE_GROUPS.keys():
        role_selection_args.add_argument(
            f"--unset_{group}_roles",
            dest="unset_roles",
            action=ExtendConstAction,
            const=ROLE_GROUPS[group],
        )
    role_selection_args.add_argument(
        "--set_roles",
        dest="set_roles",
        action="extend",
        default=[],
        nargs="+",
        help="Enable roles matching provided SET_ROLES (supports globbing)",
    )
    for group in ROLE_GROUPS.keys():
        role_selection_args.add_argument(
            f"--set_{group}_roles",
            dest="set_roles",
            action=ExtendConstAction,
            const=ROLE_GROUPS[group],
        )
    command.add_argument("args", nargs="...", help="the arguments to pass through to cmake")


@run_on_docker
def run(interactive, set_roles, unset_roles, args, **kwargs):
    pty = WrapPty()

    default_args = [
        "-DCMAKE_TOOLCHAIN_FILE=/usr/local/toolchain.cmake",
        "-DCMAKE_C_COMPILER_LAUNCHER=/usr/bin/ccache",
        "-DCMAKE_CXX_COMPILER_LAUNCHER=/usr/bin/ccache",
    ]

    # To pass arguments to the cmake command you put them after "--"
    # but "--"  isn't a valid argument for cmake, so we remove it here
    if "--" in args:
        args.remove("--")

    for pattern in unset_roles:
        role_found = False
        for role in ROLE_GROUPS["all"]:
            if glob.fnmatch.fnmatch(role, pattern):
                args.append(f"-DROLE_{role}:BOOL=off")
                role_found = True
                break
        if not role_found:
            raise RuntimeError(f"No roles matching {pattern}")

    for pattern in set_roles:
        role_found = False
        for role in ROLE_GROUPS["all"]:
            if glob.fnmatch.fnmatch(role, pattern):
                args.append(f"-DROLE_{role}:BOOL=on")
                role_found = True
                break
        if not role_found:
            raise RuntimeError(f"No roles matching {pattern}")

    # If interactive then run ccmake else just run cmake
    os.chdir(os.path.join(b.project_dir, "..", "build"))
    if interactive:
        exit(pty.spawn(["ccmake", "-GNinja", *default_args, *args, b.project_dir]))
    else:
        exit(pty.spawn(["cmake", "-GNinja", *default_args, *args, b.project_dir]))
