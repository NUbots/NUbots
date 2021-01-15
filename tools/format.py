#!/usr/bin/env python3

import multiprocessing
import os
import sys
from subprocess import STDOUT, CalledProcessError, check_output

import b
from dockerise import run_on_docker


def _do_format(path):
    text = ""
    try:
        if path.endswith((".h", ".c", ".cc", ".cxx", ".cpp", ".hpp", ".ipp", ".proto", ".frag", ".glsl", ".vert")):
            text = "Formatting {} with clang-format\n".format(path)
            text = text + check_output(["clang-format", "-i", "-style=file", path], stderr=STDOUT).decode("utf-8")
        elif path.endswith(("CMakeLists.txt", ".cmake", ".role")):
            text = "Formatting {} with cmake-format\n".format(path)
            text = text + check_output(["cmake-format", "-i", path], stderr=STDOUT).decode("utf-8")
        elif path.endswith((".py")):
            text = "Formatting {} with isort and black\n".format(path)
            text = text + check_output(["isort", path], stderr=STDOUT).decode("utf-8")
            text = text + check_output(["black", path], stderr=STDOUT).decode("utf-8")
    except CalledProcessError as e:
        text = text + e.output.decode("utf-8")

    return text


@run_on_docker
def register(command):
    command.help = "Format all the code in the codebase using clang-format"


@run_on_docker
def run(**kwargs):

    # Change into the project directory
    os.chdir(b.project_dir)

    # Use git to get all of the files that are committed to the repository
    files = check_output(["git", "ls-files"]).decode("utf-8")

    with multiprocessing.Pool(multiprocessing.cpu_count()) as pool:
        for r in pool.imap_unordered(_do_format, files.splitlines()):
            sys.stdout.write(r)
