#!/usr/bin/env python3

import os
import fnmatch
import glob
import hashlib
import tempfile
import b

from termcolor import cprint
from subprocess import call, STDOUT

try:

    def register(command):

        # Install help
        command.help = "Install the system onto the target system"

        # Drone arguments
        command.add_argument("ip_addr", metavar="ip_addr", help="the IP address of the target to install to")

        command.add_argument("hostname", metavar="hostname", help="the hostname of the target to install to")

        command.add_argument(
            "-c",
            "--config",
            metavar="config",
            choices=["", "new", "n", "update", "u", "overwrite", "o", "pull", "p", "ignore", "i"],
            default="new",
            help="method to use for configuration files when installing",
        )

        command.add_argument(
            "-s",
            "--scripts",
            metavar="scripts",
            choices=["", "new", "n", "update", "u", "overwrite", "o", "pull", "p", "ignore", "i"],
            default="new",
            help="method to use for script files when installing",
        )

        command.add_argument(
            "-u", "--user", metavar="username", default="hive", help="the username to use when installing"
        )

        command.add_argument("-t", "--toolchain", dest="toolchain", action="store_true")

    def run(ip_addr, hostname, config, scripts, user, toolchain, **kwargs):

        # Target location to install to
        target_dir = "{0}@{1}:/home/{0}/".format(user, ip_addr)
        build_dir = b.binary_dir
        config_dir = os.path.join(build_dir, ".", "config")
        script_dir = os.path.join(build_dir, ".", "scripts")
        platform_dir = "/nubots/toolchain/{0}".format(b.cmake_cache["PLATFORM"])
        roles = b.cmake_cache["NUCLEAR_ROLES"]

        cprint("Installing binaries to " + target_dir, "blue", attrs=["bold"])
        files = glob.glob(os.path.join(build_dir, "bin", "*"))
        call(["rsync", "-avzPl", "--checksum", "-e ssh"] + files + [target_dir])

        if toolchain:
            # Get all of our required shared libraries in our toolchain and send them
            # Only send toolchain files if ours are newer than the receivers.
            cprint("Installing toolchain library files", "blue", attrs=["bold"])
            libs = glob.glob("{0}/lib/*.so*".format(platform_dir))
            call(["rsync", "-avzuPl", "--checksum", "-e ssh"] + libs + [target_dir + "toolchain"])

            # Set rpath for all libs on the remote machine
            cprint(
                "Setting rpath for all toolchain libs to {0}".format(target_dir + "toolchain"), "blue", attrs=["bold"]
            )
            command = "for lib in /home/{0}/toolchain/*.so*; do patchelf --set-rpath /home/{0}/toolchain $lib; done".format(
                user
            )
            host = "{0}@{1}".format(user, ip_addr)
            cprint("Running {0} on {1}".format(command, host), "blue", attrs=["bold"])
            FNULL = open(os.devnull, "w")
            call(["ssh", host, command], stdout=FNULL, stderr=STDOUT)
            FNULL.close()

        # Get list of config files.
        config_files = [os.path.relpath(c, build_dir) for c in b.cmake_cache["NUCLEAR_MODULE_DATA_FILES"]]

        # Get list of config files.
        if config in ["overwrite", "o"]:
            cprint("Overwriting configuration files on target", "blue", attrs=["bold"])
            call(["rsync", "-avzPLR", "--checksum", "-e ssh"] + config_files + [target_dir])

        if config in ["update", "u"]:
            cprint("Adding new configuration files to target", "blue", attrs=["bold"])
            call(["rsync", "-avzuPLR", "--checksum", "-e ssh"] + config_files + [target_dir])

        if not config or config in ["new", "n"]:
            cprint("Adding new configuration files to the target", "blue", attrs=["bold"])
            call(["rsync", "-avzPLR", "--checksum", "--ignore-existing", "-e ssh"] + config_files + [target_dir])

        if config in ["ignore", "i"]:
            cprint("Ignoring configuration changes", "blue", attrs=["bold"])

        # Pipe the git commit to file
        with open(os.path.join(build_dir, "version.txt"), "w") as f:
            call(["git", "log", "-1"], stdout=f)

        call(["rsync", "-avzPLR", "--checksum", "-e ssh"] + ["version.txt"] + [target_dir])


except:
    print("Unable to load install tool")

    def register(command):
        pass

    def run(**kwargs):
        pass
