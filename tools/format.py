#!/usr/bin/env python3

import multiprocessing
import os
import sys
from functools import partial
from subprocess import STDOUT, CalledProcessError, check_output

import b
from utility.dockerise import run_on_docker
from utility.yarn import find_eslint, find_package_json


def _do_format(path, eslint_path, package_path, skip_typescript, skip_cpp, skip_protobuf, skip_python, skip_cmake):
    text = ""
    try:
        # Use the absolute path to file
        abs_path = os.path.abspath(path)

        # .frag, .glsl, and .vert files are lumped in with C++
        if not skip_cpp and path.endswith(
            (".h", ".c", ".cc", ".cxx", ".cpp", ".hpp", ".ipp", ".frag", ".glsl", ".vert")
        ):
            text = "Formatting {} with clang-format\n".format(path)
            text = text + check_output(["clang-format", "-i", "-style=file", abs_path], stderr=STDOUT).decode("utf-8")
        elif not skip_protobuf and path.endswith((".proto")):
            text = "Formatting {} with clang-format\n".format(path)
            text = text + check_output(["clang-format", "-i", "-style=file", abs_path], stderr=STDOUT).decode("utf-8")
        elif not skip_cmake and path.endswith(("CMakeLists.txt", ".cmake", ".role")):
            text = "Formatting {} with cmake-format\n".format(path)
            text = text + check_output(["cmake-format", "-i", abs_path], stderr=STDOUT).decode("utf-8")
        elif not skip_python and path.endswith((".py")):
            text = "Formatting {} with isort and black\n".format(path)
            text = text + check_output(["isort", abs_path], stderr=STDOUT).decode("utf-8")
            text = text + check_output(["black", abs_path], stderr=STDOUT).decode("utf-8")
        elif eslint_path is not None and not skip_typescript and (path.endswith((".ts")) or path.endswith((".tsx"))):
            text = "Formatting {} with eslint and prettier\n".format(path)
            text = text + check_output(
                [eslint_path, "--color", "--fix", abs_path], cwd=package_path, stderr=STDOUT
            ).decode("utf-8")
    except CalledProcessError as e:
        text = text + e.output.decode("utf-8")

    return text


@run_on_docker
def register(command):
    command.help = "Format all the code in the codebase using clang-format"

    command.add_argument(
        "-t",
        "--skip-typescript",
        dest="skip_typescript",
        action="store_true",
        default=False,
        help="Skip formatting the typescript",
    )
    command.add_argument(
        "-c",
        "--skip-cpp",
        dest="skip_cpp",
        action="store_true",
        default=False,
        help="Skip formatting the C++",
    )
    command.add_argument(
        "-g",  # g for google protobuf. p is already taken for python
        "--skip-protobuf",
        dest="skip_protobuf",
        action="store_true",
        default=False,
        help="Skip formatting the protobuf",
    )
    command.add_argument(
        "-p",
        "--skip-python",
        dest="skip_python",
        action="store_true",
        default=False,
        help="Skip formatting the python",
    )
    command.add_argument(
        "-m",
        "--skip-cmake",
        dest="skip_cmake",
        action="store_true",
        default=False,
        help="Skip formatting the cmake",
    )
    command.add_argument(
        "-u",
        "--format-unchanged",
        dest="format_unchanged",
        action="store_true",
        default=False,
        help="Include unmodified files, as well as modified files, compared to main.",
    )


@run_on_docker
def run(skip_typescript, skip_cpp, skip_protobuf, skip_python, skip_cmake, format_unchanged, **kwargs):
    # Check for eslint in node_modules folder
    eslint_path = find_eslint(os.path.join(b.project_dir, "nusight2", "node_modules"))

    # Find path to package.json
    # We need to run the eslint command from this directory
    package_path = find_package_json(project_root=b.project_dir, package_name="NUsight")

    # Change into the project directory
    os.chdir(b.project_dir)

    # Use git to get all of the files that are committed to the repository
    files = (
        check_output(["git", "ls-files"]).decode("utf-8")
        if format_unchanged
        else check_output(["git", "diff", "--name-only", "main"]).decode("utf-8")
    )

    with multiprocessing.Pool(multiprocessing.cpu_count()) as pool:
        for r in pool.imap_unordered(
            partial(
                _do_format,
                eslint_path=eslint_path,
                package_path=package_path,
                skip_typescript=skip_typescript,
                skip_cpp=skip_cpp,
                skip_protobuf=skip_protobuf,
                skip_python=skip_python,
                skip_cmake=skip_cmake,
            ),
            files.splitlines(),
        ):
            sys.stdout.write(r)
