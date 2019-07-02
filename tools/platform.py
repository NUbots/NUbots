#!/usr/bin/env python3

import os
import b
import argparse
from termcolor import cprint
from subprocess import call, STDOUT

try:

    def register(command):

        # Install help
        command.help = "Tools to work with building the codebase"

        # Module subcommands
        subcommands = command.add_subparsers(dest="workspace_command")

        init_command = subcommands.add_parser("select", help="Choose a specific platform as the main platform")

        toolchains = []
        # Work out what platforms are available
        if os.path.isdir("/nubots/toolchain"):
            for f in os.listdir("/nubots/toolchain/"):
                if f.endswith(".cmake"):
                    toolchains.append(f[:-6])

        init_command.add_argument(
            "platform", metavar="platform", choices=toolchains, help="the platform to select as the primary workspace"
        )

        init_command.add_argument(
            "-s",
            "--static",
            dest="static",
            action="store_true",
            help="perform a static build to get maximum performance",
        )

        init_command.add_argument("cmake_args", nargs=argparse.REMAINDER, help="Extra arguments to pass to cmake")

    def run(workspace_command, static=False, cmake_args=None, **kwargs):
        if workspace_command == "select":
            platform = kwargs["platform"]

            # init and update submodules just in case
            os.system("git submodule init")
            os.system("git submodule update")

            print("Activating workspace as platform {}".format(platform))
            path = os.path.join(b.project_dir, "build_{}".format(platform))

            # Make the directory
            try:
                os.makedirs(path)
            except FileExistsError:
                pass

            # Make the symlink to make this the main directory
            try:
                os.remove("build")
            except FileNotFoundError:
                pass

            # Try to make the symlink, this will fail if you use windows
            symlink_success = True
            try:
                os.symlink(path, "build")
            except OSError:
                symlink_success = False

            # Change to that directory
            os.chdir(path)

            # Build our default args
            args = [
                "cmake",
                b.project_dir,
                "-GNinja",
                "-DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/{}.cmake".format(platform),
            ]

            if static:
                args.append("-DSTATIC_LIBRARIES=ON")
                args.append("-DNUCLEAR_SHARED_BUILD=OFF")

            if cmake_args is not None:
                args += cmake_args

            # Run cmake
            call(args)

            # Yell at windows users for having a crappy OS
            if not symlink_success:
                cprint(
                    "Windows does not support symlinks so we can't link to the build directory", "red", attrs=["bold"]
                )
                cprint(
                    "Instead you will need to change to the build_{} directory to build".format(platform),
                    "red",
                    attrs=["bold"],
                )


except:
    print("Unable to load platform tool")

    def register(command):
        pass

    def run(**kwargs):
        pass
