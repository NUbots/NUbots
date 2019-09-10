#!/usr/bin/env python3

import os
import glob
import b
import re
from dockerise import run_on_docker

from termcolor import cprint
import subprocess


@run_on_docker
def register(command):
    command.help = "Install the system onto the target system"

    # Configuration
    command.add_argument("target", help="the target host to install the packages to")

    command.add_argument("--user", "-u", help="the user to install onto the target with")

    command.add_argument(
        "--config",
        "-c",
        choices=["new", "n", "update", "u", "overwrite", "o", "ignore", "i"],
        default="new",
        help="method to use for configuration files when installing",
    )

    command.add_argument("-t", "--toolchain", dest="toolchain", action="store_true")


@run_on_docker
def run(target, user, config, toolchain, **kwargs):

    # Replace hostname with its IP address if the hostname is already known
    num_robots = 4
    target = {
        "{}{}".format(k, num): "10.1.1.{}".format(num)
        for num in range(1, num_robots + 1)
        for k, v in zip(("nugus", "n", "i", "igus"), [num] * num_robots)
    }.get(target, target)

    # If no user, use our user
    if user is None:
        import getpass

        user = getpass.getuser()

    # Target location to install to
    target_dir = "{0}@{1}:/home/{0}/".format(user, target)
    build_dir = b.binary_dir

    cprint("Installing binaries to " + target_dir, "blue", attrs=["bold"])
    files = glob.glob(os.path.join(build_dir, "bin", "*"))
    subprocess.call(["rsync", "-avzPl", "--checksum", "-e ssh"] + files + [target_dir])

    if toolchain:
        # Get all of our required shared libraries in our toolchain and send them
        # Only send toolchain files if ours are newer than the receivers.
        # Delete toolchain files on the receiver if they no longer exist in our toolchain
        cprint("Installing toolchain library files", "blue", attrs=["bold"])

        subprocess.call(
            [
                "rsync",
                "-avzlP",
                "--include=local",
                "--include=local/lib",
                "--include=local/lib/**/",
                "--include=local/lib/**.so",
                "--include=local/lib/**.so.*",
                "--include=local/lib/python3.7/**",
                "--exclude=*",
                "--checksum",
                "--delete",
                "--prune-empty-dirs",
                "-e ssh",
                "/usr/local",
                "{0}@{1}:/usr/".format(user, target),
            ]
        )

        # Run ldconfig on the robot to ensure the system knows that the new libraries are there
        cprint("Running ldconfig on {}".format(target), "blue", attrs=["bold"])
        subprocess.call(["ssh", "{}@{}".format(user, target), "sudo ldconfig"])

    # If there is only a single file then the b script returns this as a string rather than a list
    config_files = b.cmake_cache["NUCLEAR_MODULE_DATA_FILES"]
    config_files = config_files if isinstance(config_files, list) else [config_files]

    # Get list of config files
    config_files = [os.path.relpath(c, build_dir) for c in config_files]

    # Change to the build directory so our relative paths are correct
    os.chdir(build_dir)

    if config in ["overwrite", "o"]:
        cprint("Overwriting configuration files on target", "blue", attrs=["bold"])
        subprocess.call(["rsync", "-avzPLR", "--checksum", "-e ssh"] + config_files + [target_dir])

    if config in ["update", "u"]:
        cprint("Updating configuration files that are older on target", "blue", attrs=["bold"])
        subprocess.call(["rsync", "-avzuPLR", "--checksum", "-e ssh"] + config_files + [target_dir])

    if config in ["new", "n"]:
        cprint("Adding new configuration files to the target", "blue", attrs=["bold"])
        subprocess.call(["rsync", "-avzPLR", "--checksum", "--ignore-existing", "-e ssh"] + config_files + [target_dir])

    if config in ["ignore", "i"]:
        cprint("Ignoring configuration changes", "blue", attrs=["bold"])

    # Pipe the git commit to file
    version_file = os.path.join(build_dir, "version.txt")
    with open(version_file, "w") as f:
        os.chdir(b.project_dir)
        subprocess.call(["git", "log", "-1"], stdout=f)

    subprocess.call(["scp", version_file, target_dir])
