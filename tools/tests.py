#!/usr/bin/env python3

# TODO: add ability to list tests like `./b tests list` (ctest -N)
# TODO: add ability to run individual test like `./b tests run <test_name>`

import os
import time
import subprocess
import shlex
import multiprocessing

import b
from dockerise import WrapPty, run_on_docker

# File cannot be named `test.py` for some reason?

# Directory to create in project root for storing output of tests
TESTS_OUTPUT_DIR = "tests_output"


@run_on_docker
def register(command):

    command.help = "Run tests"

    command.add_argument(
        "-VV",
        "--extra-verbose",
        action="append_const",
        dest="given_ctest_args",
        const="-VV",
        help="Enable more verbose output from tests",
    )

    command.add_argument(
        "-V",
        "--verbose",
        action="append_const",
        dest="given_ctest_args",
        const="-V",
        help="Enable verbose output from tests",
    )

    command.add_argument(
        "-Q",
        "--quiet",
        action="append_const",
        dest="given_ctest_args",
        const="-Q",
        help="Make ctest not print to stdout",
    )

    command.add_argument(
        "-j",
        "--parallel",
        action="store",
        dest="num_jobs",
        default=multiprocessing.cpu_count(),
        help="Run the tests in parallel using the given number of jobs.",
    )

    command.add_argument(
        "--debug",
        action="append_const",
        dest="given_ctest_args",
        const="--debug",
        help="Displaying more verbose internals of CTest",
    )


@run_on_docker
def run(given_ctest_args, num_jobs, **kwargs):
    tests_dir = os.path.join(b.project_dir, TESTS_OUTPUT_DIR)

    # If tests dir not at /home/nubots/Nubots/tests , create it
    if not os.path.exists(tests_dir):
        os.makedirs(tests_dir)

    # Change into the build directory
    os.chdir(os.path.join(b.project_dir, "..", "build"))

    # Windows friendly (container time, not host)
    filename = time.strftime("%Y-%m-%d-%H-%M-%S") + ".log"
    logPath = os.path.join(tests_dir, filename)

    # Default ctest args
    ctest_args = [
        "--force-new-ctest-process",
        "--output-on-failure",
    ]

    # Add given args to default args
    if given_ctest_args:
        ctest_args.extend(given_ctest_args)

    exit(
        subprocess.run(["/usr/bin/ctest", "--output-log", logPath, "--parallel", str(num_jobs), *ctest_args]).returncode
    )
