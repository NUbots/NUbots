#!/usr/bin/env python3

import os
import time
import subprocess
import shlex

import b
from dockerise import WrapPty, run_on_docker

# File cannot be named `test.py` for some reason?


@run_on_docker
def register(command):

    command.help = "Run tests"

    command.add_argument(
        "-VV", action="append_const", dest="argss", const="-VV", help="Enable more verbose output from tests"
    )

    command.add_argument(
        "-V", action="append_const", dest="ctest_args", const="-V", help="Enable verbose output from tests"
    )

    command.add_argument("-Q", action="append_const", dest="ctest_args", const="-Q", help="Make ctest quiet")

    command.add_argument(
        "-J",
        action="store",
        dest="jobs",
        default=1,
        help="Run the tests in parallel using the given number of jobs.",
    )


@run_on_docker
def run(ctest_args, jobs, **kwargs):
    tests_dir = os.path.join(b.project_dir, "tests")

    # If tests dir not at /home/nubots/Nubots/tests , create it
    if not os.path.exists(tests_dir):
        os.makedirs(tests_dir)

    # Change into the build directory
    os.chdir(os.path.join(b.project_dir, "..", "build"))
    filename = time.strftime("%Y-%m-%d-%H-%M-%S") + ".log"  # Windows friendly (container time, not host)
    logPath = os.path.join(tests_dir, filename)

    if not ctest_args:
        ctest_args = [
            "--force-new-ctest-process",
            "--output-on-failure",
        ]
        exit(
            subprocess.run(["/usr/bin/ctest", "--output-log", logPath, "--parallel", str(jobs), *ctest_args]).returncode
        )

    else:
        exit(
            subprocess.run(["/usr/bin/ctest", "--output-log", logPath, "--parallel", str(jobs), *ctest_args]).returncode
        )
