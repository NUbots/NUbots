#!/usr/bin/env python3

import glob
import os
import subprocess

from termcolor import cprint

import b
from utility.dockerise import run_on_docker


@run_on_docker
def register(command):
    command.description = "Install the system onto the target system"

    command.add_argument("target", help="The target host or directory to install the packages to")

    command.add_argument("--local", "-l", dest="local", action="store_true", help="Install to a local directory")

    command.add_argument("--user", "-u", help="The user to install onto the target with")

    command.add_argument(
        "--config",
        "-c",
        choices=["new", "n", "update", "u", "overwrite", "o", "ignore", "i"],
        default="new",
        help="The method to use for configuration files when installing",
    )

    command.add_argument("-t", "--toolchain", dest="toolchain", action="store_true")


@run_on_docker
def run(target, local, user, config, toolchain, **kwargs):
    # Replace hostname with its IP address if the hostname is already known
    if not local:
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
    if local:
        target_binaries_dir = os.path.abspath(os.path.join(target, f"binaries{os.sep}"))
        target_toolchain_dir = os.path.abspath(os.path.join(target, f"toolchain{os.sep}"))

        # Ensure the directories exist
        os.makedirs(target_binaries_dir, exist_ok=True)
        os.makedirs(target_toolchain_dir, exist_ok=True)
    else:
        target_binaries_dir = "{0}@{1}:/home/{0}/".format(user, target)
        target_toolchain_dir = "{0}@{1}:/usr/".format(user, target)

    # Build directory on the robot
    build_dir = b.binary_dir

    # Recursively gather all files under build/bin
    cprint("Installing binaries to " + target_binaries_dir, "blue", attrs=["bold"])
    files = glob.glob(os.path.join(build_dir, "bin", "**", "*"), recursive=True)

    # Add a /./ to files so rsync --relative/-R behaves how we want it to
    # For example, /home/NUbots/build/bin/binary will become /home/NUbots/build/bin/./binary
    common_path = os.path.commonpath(files)
    files = [os.path.join(common_path, f.replace(common_path, ".")) for f in files]
    subprocess.call(["rsync", "-avPlR", "--checksum", "-e ssh"] + files + [target_binaries_dir])

    if toolchain:
        # Get all of our required shared libraries in our toolchain and send them
        # Only send toolchain files if ours are newer than the receivers.
        # Delete toolchain files on the receiver if they no longer exist in our toolchain
        cprint("Installing toolchain files to " + target_toolchain_dir, "blue", attrs=["bold"])

        subprocess.run(
            [
                "rsync",
                "-avPl",
                "--include=local",
                "--include=local/lib",
                "--include=local/lib/**/",
                "--include=local/lib/**.so",
                "--include=local/lib/**.so.*",
                "--include=local/lib/python3.7/**",
                "--include=local/sbin",
                "--include=local/sbin/**",
                "--include=local/share",
                "--include=local/share/**",
                "--exclude=*",
                "--checksum",
                "--delete",
                "--prune-empty-dirs",
                "-e ssh",
                "/usr/local",
                target_toolchain_dir,
            ]
        )

        # Run ldconfig on the robot to ensure the system knows that the new libraries are there
        if not local:
            cprint("Running ldconfig on {}".format(target), "blue", attrs=["bold"])
            subprocess.run(["ssh", "{}@{}".format(user, target), "sudo ldconfig"])

    # Get list of different config files for concatenation
    script_files = b.cmake_cache["SCRIPT_FILES"]
    model_files = b.cmake_cache["MODEL_FILES"]
    data_files = b.cmake_cache["NUCLEAR_MODULE_DATA_FILES"]

    # If there is only a single file then the b script returns this as a string rather than a list
    # Handle this by forcing into a list if it is not already
    config_files = [data_files] if not isinstance(data_files, list) else data_files
    config_files += [script_files] if not isinstance(script_files, list) else script_files
    config_files += [model_files] if not isinstance(model_files, list) else model_files
    config_files = config_files if isinstance(config_files, list) else [config_files]

    # Get list of config files
    config_files = [os.path.relpath(c, build_dir) for c in config_files]

    # Change to the build directory so our relative paths are correct
    os.chdir(build_dir)

    if config in ["overwrite", "o"]:
        cprint("Overwriting configuration files on target", "blue", attrs=["bold"])
        subprocess.run(["rsync", "-avPLR", "--checksum", "-e ssh"] + config_files + [target_binaries_dir])

    if config in ["update", "u"]:
        cprint("Updating configuration files that are older on target", "blue", attrs=["bold"])
        subprocess.run(["rsync", "-avuPLR", "--checksum", "-e ssh"] + config_files + [target_binaries_dir])

    if config in ["new", "n"]:
        cprint("Adding new configuration files to the target", "blue", attrs=["bold"])
        subprocess.run(
            ["rsync", "-avPLR", "--checksum", "--ignore-existing", "-e ssh"] + config_files + [target_binaries_dir]
        )

    if config in ["ignore", "i"]:
        cprint("Ignoring configuration changes", "blue", attrs=["bold"])

    # Pipe the git commit to file
    version_file = os.path.join(build_dir, "version.txt")
    with open(version_file, "w") as f:
        os.chdir(b.project_dir)
        subprocess.run(["git", "log", "-1", "--pretty=format:'%H'"], stdout=f)

    subprocess.run(["scp", version_file, target_binaries_dir])
