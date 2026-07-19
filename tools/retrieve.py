#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2026 NUbots
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

import os
import shutil
import subprocess

from termcolor import cprint

from utility.dockerise import run_on_docker

TEMP_FOLDER = "temp"
CONFIG_FOLDER = "/home/nubots/config"
RECORDINGS_FOLDER = "/home/nubots/recordings"
SCRIPTS_FOLDER = "ican'trememberwherethisgoes"


@run_on_docker
def register(command):
    command.description = "Retrieve files from the target system to the local system"

    command.add_argument("host", help="The host to retrieve the files from")

    command.add_argument(
        "target",
        help="The target to retrieve: config/recordings/scripts/the_works",
    )

    command.add_argument("--user", "-u", help="The user to retrieve the files with", default="nubots")


@run_on_docker
def run(host, target, user=None, **kwargs):
    # Replace hostname with its IP address if the hostname is already known
    num_robots = 4
    host = {
        "{}{}".format(prefix, num): "10.1.1.{}".format(num)
        for num in range(1, num_robots + 1)
        for prefix in ("nugus", "n", "i", "igus")
    }.get(host, host)

    if target == "config":
        """
        STAGE 1:
        Copy all the configs back from the robot
        """

        cprint(f"Retrieving config from {host}:{CONFIG_FOLDER} to {TEMP_FOLDER}/", "green")

        subprocess.run(  # this retrieves back to a temp folder - just for now
            [
                "rsync",
                "-aP",  # partial progression, archive mode
                "-e",  # specify the remote shell to use
                "ssh",
                f"{user}@{host}:{CONFIG_FOLDER}",
                f"{TEMP_FOLDER}/",
            ],
            check=True,
        )

        # list every file in TEMP_FOLDER

        if not os.path.isdir(TEMP_FOLDER):
            cprint(f"Temp folder '{TEMP_FOLDER}' does not exist", "red")
            return

        files_in_temp = []
        for root, _, filenames in os.walk(TEMP_FOLDER):
            for fn in filenames:
                files_in_temp.append(os.path.relpath(os.path.join(root, fn), TEMP_FOLDER))

        if not files_in_temp:
            cprint(f"No files found in '{TEMP_FOLDER}'", "yellow")
        else:
            cprint(f"Files in '{TEMP_FOLDER}':", "green")
            print(files_in_temp)

        all_matched = True
        for temp_file in files_in_temp:
            matches = []
            for root, _, filenames in os.walk(os.getcwd()):
                # skip the temp folder itself, otherwise every file matches its own copy
                if os.path.abspath(root).startswith(os.path.abspath(TEMP_FOLDER)):
                    continue
                if os.path.basename(temp_file) in filenames:
                    matches.append(os.path.join(root, os.path.basename(temp_file)))
            if matches:
                cprint(f"{temp_file} found in {len(matches)} place(s):", "cyan")
                for match in matches:
                    print(f"  {match}")

                temp_parent_folder = os.path.basename(os.path.dirname(temp_file))

                # first sort: parent folder name matches (ie. frankie/SomeConfig.yaml)
                best_match = next((m for m in matches if os.path.basename(os.path.dirname(m)) == temp_parent_folder), None)
                if best_match:
                    print("Match for common parent folder name:", temp_parent_folder)

                    shutil.copy(os.path.join(TEMP_FOLDER, temp_file), best_match)

                # failing this...
                # second sort: parent folder's name of match is config (ie. it's not robot specific)
                if not best_match:
                    best_match = next((m for m in matches if os.path.basename(os.path.dirname(m)) == "config"), None)
                    if best_match:
                        print("Match for generic config.")

                        shutil.copy(os.path.join(TEMP_FOLDER, temp_file), best_match)

                # failing this...
                # tell the user we couldn't find a match
                if not best_match:
                    print("Unable to match.")
                    all_matched = False
            else:
                cprint(f"No local match found for {temp_file}", "yellow")
                all_matched = False

        # now remove all the temp files, but only if everything found a home
        if all_matched:
            shutil.rmtree(TEMP_FOLDER)
        else:
            cprint(f"Some files could not be matched — keeping '{TEMP_FOLDER}' for manual handling", "yellow")


    if target == "recordings":
        """
        STAGE 2:
        Copy all the recordings back ONLY if the user wants it (these can be chunky bois)
        """
