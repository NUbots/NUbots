#!/usr/bin/env python3
import multiprocessing
import os
import sys
from fnmatch import fnmatch
from functools import partial
from subprocess import PIPE, STDOUT, CalledProcessError
from subprocess import run as sp_run

import b
from utility.dockerise import run_on_docker

# The extensions that are handled by the various formatters
formatters = {
    "clang-format": {
        "format": [["clang-format", "-i", "-style=file"]],
        "check": [["clang-format", "-style=file"]],
        "include": ["*.h", "*.c", "*.cc", "*.cxx", "*.cpp", "*.hpp", "*.ipp", "*.frag", "*.glsl", "*.vert", "*.proto"],
        "exclude": [],
    },
    "cmake-format": {
        "format": [["cmake-format", "--in-place"]],
        "check": [["cmake-format"]],
        "include": ["*.cmake", "*.role", "CMakeLists.txt", "**/CMakeLists.txt"],
        "exclude": [],
    },
    "isort and black": {
        "format": [["isort"], ["black", "--quiet"]],
        "check": [["isort", "--py", "auto", "-d"], ["black", "-"]],
        "include": ["*.py"],
        "exclude": [],
    },
    "prettier": {
        "format": [["prettier", "--write"]],
        "check": [["prettier"]],
        "include": ["*.js", "*.jsx", "*.ts", "*.tsx", "*.json", "*.css", "*.scss", "*.html", "*.md", "*.yaml", "*.yml"],
        "exclude": ["*.min.*"],
    },
}


def _do_check(path, verbose):
    text = "" if not verbose else f"Skipping {path} as it does not match any of the formatters\n"
    success = True
    try:
        # Find the correct formatter and format the file
        for name, fmt in formatters.items():
            if (any(fnmatch(path, pattern) for pattern in fmt["include"])) and (
                all(not fnmatch(path, pattern) for pattern in fmt["exclude"])
            ):
                text = f"Checking formatting of {path} with {name}\n"

                # Format the code
                run_args = {"capture_output": True, "check": True}
                cmd = sp_run(fmt["check"][0] + [path], **run_args)
                for c in fmt["check"][1:]:
                    cmd = sp_run(c, input=cmd.stdout, **run_args)

                # Run the diff command
                cmd = sp_run(["colordiff", "--color=yes", "-u", path, "-"], input=cmd.stdout, **run_args)
                tool_text = cmd.stdout.decode("utf-8").strip()

                text = text + tool_text if tool_text or verbose else ""
                break

    except CalledProcessError as e:
        text = text + e.output.decode("utf-8")
        success = False

    return text, success


def _do_format(path, verbose):
    text = "" if not verbose else f"Skipping {path} as it does not match any of the formatters\n"
    success = True
    try:
        # Find the correct formatter and format the file
        for name, fmt in formatters.items():
            if (any(fnmatch(path, pattern) for pattern in fmt["include"])) and (
                all(not fnmatch(path, pattern) for pattern in fmt["exclude"])
            ):
                text = f"Formatting {path} with {name}\n"

                # Format the code
                run_args = {"stderr": STDOUT, "stdout": PIPE, "check": True}
                tool_text = ""
                for c in fmt["format"]:
                    tool_text = tool_text + sp_run(c + [path], **run_args).stdout.decode("utf-8")

                text = text + tool_text if tool_text or verbose else ""
                break

    except CalledProcessError as e:
        text = text + e.output.decode("utf-8")
        success = False

    return text, success


@run_on_docker
def register(command):
    command.help = "Format all the code in the codebase using clang-format"

    command.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        default=False,
        help="Print the output of the formatters",
    )
    command.add_argument(
        "-a",
        "--all",
        dest="format_all",
        action="store_true",
        default=False,
        help="Include unmodified files, as well as modified files, compared to main.",
    )
    command.add_argument(
        "-c",
        "--check",
        dest="check",
        action="store_true",
        default=False,
        help="Check that files conform to formatting requirements",
    )
    command.add_argument(
        "globs",
        nargs="*",
        help="Globs with which to limit the files to format",
    )


@run_on_docker
def run(verbose, check, format_all, globs, **kwargs):

    # Change into the project directory
    os.chdir(b.project_dir)

    # Use git to get all of the files that are committed to the repository or just the ones that are different to main
    if format_all:
        files = sp_run(["git", "ls-files"], stdout=PIPE, check=True).stdout.decode("utf-8").splitlines()
    else:
        files = (
            sp_run(["git", "diff", "--name-only", "main"], stdout=PIPE, check=True).stdout.decode("utf-8").splitlines()
        )

    # Filter to a list containing only existing files (git diff can return deleted files)
    files = [f for f in files if os.path.isfile(f)]

    # Filter the files we found by any globs we are using
    if len(globs) != 0:

        def matches_glob(f):
            for g in globs:
                if fnmatch(f, g):
                    return True
            return False

        files = filter(matches_glob, files)

    _action_func = _do_format if not check else _do_check
    success = True
    with multiprocessing.Pool(multiprocessing.cpu_count()) as pool:
        for r, s in pool.imap_unordered(partial(_action_func, verbose=verbose), files):
            sys.stdout.write(r)
            success = success and s

    sys.exit(0 if success else 1)
