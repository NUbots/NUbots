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
SCRIPTS_FOLDER = "/home/nubots/scripts"

TARGETS = ("config", "recordings", "scripts", "the_works")


@run_on_docker
def register(command):
    command.description = "Retrieve files from the target system to the local system"

    command.add_argument("host", help="The host to retrieve the files from")

    command.add_argument(
        "target",
        choices=TARGETS,
        help="The target to retrieve: config/recordings/scripts/the_works",
    )

    command.add_argument("--user", "-u", help="The user to retrieve the files with", default="nubots")


def rsync_from(host, user, remote_folder, local_folder):
    subprocess.run(
        [
            "rsync",
            "-aP",  # partial progression, archive mode
            "-e",  # specify the remote shell to use
            "ssh",
            # trailing slash on the source makes rsync copy the folder's contents,
            # not the folder itself (avoids recordings/recordings/)
            f"{user}@{host}:{remote_folder.rstrip('/')}/",
            f"{local_folder}/",
        ],
        check=True,
    )


def retrieve_and_merge(host, user, remote_folder, generic_parent):
    """
    Retrieve remote_folder into TEMP_FOLDER, copy each file over its best local
    match (parent folder name first, then generic_parent), then delete TEMP_FOLDER.
    """
    cprint(f"Retrieving from {host}:{remote_folder} to {TEMP_FOLDER}/", "green")

    rsync_from(host, user, remote_folder, TEMP_FOLDER)

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
            # second sort: parent folder's name of match is the generic one (ie. it's not robot specific)
            if not best_match:
                best_match = next((m for m in matches if os.path.basename(os.path.dirname(m)) == generic_parent), None)
                if best_match:
                    print(f"Match for generic parent folder: {generic_parent}")

                    shutil.copy(os.path.join(TEMP_FOLDER, temp_file), best_match)

            # failing this...
            # tell the user we couldn't find a match
            if not best_match:
                print("Unable to match.")
        else:
            cprint(f"No local match found for {temp_file}", "yellow")

    # now remove all the temp files
    shutil.rmtree(TEMP_FOLDER)


def retrieve_recordings(host, user):
    """
    Copy all the recordings back ONLY if the user wants it (these can be chunky bois)
    """
    cprint(f"Retrieving recordings from {host}:{RECORDINGS_FOLDER} to recordings/", "green")

    rsync_from(host, user, RECORDINGS_FOLDER, "recordings")


@run_on_docker
def run(host, target, user=None, **kwargs):
    # Replace hostname with its IP address if the hostname is already known
    num_robots = 4
    host = {
        "{}{}".format(prefix, num): "10.1.1.{}".format(num)
        for num in range(1, num_robots + 1)
        for prefix in ("nugus", "n", "i", "igus")
    }.get(host, host)

    # STAGE 1: copy all the configs back from the robot
    if target in ("config", "the_works"):
        retrieve_and_merge(host, user, CONFIG_FOLDER, "config")

    # STAGE 2: copy all the recordings back
    if target in ("recordings", "the_works"):
        retrieve_recordings(host, user)

    # STAGE 3: copy all the scripts back, filtered very similar to the configs
    if target in ("scripts", "the_works"):
        retrieve_and_merge(host, user, SCRIPTS_FOLDER, "nugus")
