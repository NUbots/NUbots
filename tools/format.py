#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2017 NUbots
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

import datetime
import multiprocessing
import os
import shutil
import sys
import tempfile
from collections import OrderedDict
from fnmatch import fnmatch
from functools import partial
from subprocess import DEVNULL, PIPE, STDOUT, CalledProcessError
from subprocess import run as sp_run

import pygit2

import b
from utility.dockerise import run_on_docker

repo = pygit2.Repository(b.project_dir)

# The extensions that are handled by the various formatters
formatters = OrderedDict()

formatters["licence"] = {
    "format": [
        [
            "licenseheaders",
            "-t",
            os.path.join(b.project_dir, ".licence.tmpl"),
            "--years={added}",
            "--owner=NUbots",
            f"--projname=NUbots",
            "--projurl=https://github.com/NUbots/NUbots",
            "-f",
            "{path}",
        ]
    ],
    "include": [
        "*.cpp",
        "*.hpp",
        "*.h",
        "*.c",
        "*.py",
        "*.sh",
        "*.cmake",
        "CMakeLists.txt",
        "**/CMakeLists.txt",
        "Dockerfile",
        "**/Dockerfile",
    ],
    "exclude": ["shared/utility/motion/splines/*"],  # TODO exclude files that are not ours
}
formatters["clang-format"] = {
    "format": [["clang-format", "-i", "-style=file", "{path}"]],
    "include": ["*.h", "*.c", "*.cc", "*.cxx", "*.cpp", "*.hpp", "*.ipp", "*.frag", "*.glsl", "*.vert", "*.proto"],
    "exclude": [],
}
formatters["cmake-format"] = {
    "format": [["cmake-format", "--in-place", "{path}"]],
    "include": ["*.cmake", "*.role", "CMakeLists.txt", "**/CMakeLists.txt"],
    "exclude": [],
}
formatters["isort"] = {
    "format": [["isort", "--quiet", "{path}"]],
    "include": ["*.py"],
    "exclude": [],
}
formatters["black"] = {
    "format": [["black", "--quiet", "{path}"]],
    "include": ["*.py"],
    "exclude": [],
}
formatters["eslint"] = {
    "format": [["eslint", "--color", "--fix", "{path}"]],
    "include": ["*.js", "*.jsx", "*.ts", "*.tsx"],
    "exclude": ["*.min.*", "doc/**"],
}
formatters["prettier"] = {
    "format": [["prettier", "--write", "{path}"]],
    "include": ["*.js", "*.jsx", "*.ts", "*.tsx", "*.json", "*.css", "*.scss", "*.html", "*.md", "*.yaml", "*.yml"],
    "exclude": ["*.min.*"],
}


def _get_history_dates(path):
    years = []

    # Keep track of the name of the file we are looking for for each commit
    path_for_commit = {repo[repo.head.target].id: path}

    for commit in (walker := repo.walk(repo[repo.head.target].id, pygit2.GIT_SORT_TOPOLOGICAL)):
        if commit.id not in path_for_commit:
            continue

        search_path = path_for_commit[commit.id]
        relevant = len(commit.parents) == 0

        for p in commit.parents:
            # Initially keep the path the same
            path_for_commit[p.id] = search_path

            diff = p.tree.diff_to_tree(commit.tree)

            # Only bother doing a similarity search if the file was modified
            if any(delta.new_file.path == search_path for delta in diff.deltas):
                diff.find_similar(
                    flags=pygit2.GIT_DIFF_FIND_RENAMES
                    | pygit2.GIT_DIFF_FIND_RENAMES_FROM_REWRITES
                    | pygit2.GIT_DIFF_FIND_IGNORE_WHITESPACE
                )
                for delta in diff.deltas:
                    if delta.new_file.path == search_path:
                        relevant = True

                    # The file was added here, stop looking
                    if delta.status == pygit2.GIT_DELTA_ADDED and delta.new_file.path == search_path:
                        del path_for_commit[p.id]
                        walker.hide(p.id)
                    # The file was renamed, update where we are looking
                    elif delta.status == pygit2.GIT_DELTA_RENAMED and delta.new_file.path == search_path:
                        path_for_commit[p.id] = delta.old_file.path

        if relevant:
            tzinfo = datetime.timezone(datetime.timedelta(minutes=commit.author.offset))
            dt = datetime.datetime.fromtimestamp(float(commit.author.time), tzinfo)
            years.append(dt.year)

    return max(years), min(years)


# This function is used to get details of a file that might be needed in the arguments of a formatter
# For example the year the file was added and the year it was last modified for a licence header
def _get_args(path):
    modified, added = _get_history_dates(path)
    return {"added": f"{added}", "modified": f"{modified}"}


# Format or check the file
def _do_format(path, verbose, check=True):
    text = ""
    success = True
    output_path = None
    try:
        # Find the correct formatter and format the file
        formatter = []
        formatter_names = []
        for name, fmt in formatters.items():
            if (any(fnmatch(path, pattern) for pattern in fmt["include"])) and (
                all(not fnmatch(path, pattern) for pattern in fmt["exclude"])
            ):
                formatter_names.append(name)
                formatter.extend(fmt["format"])

        # If we don't have a formatter then skip this file
        if len(formatter) == 0:
            return f"Skipping {path} as it does not match any of the formatters\n" if verbose >= 1 else "", True

        text = f"Formatting {path} with {', '.join(formatter_names)}\n"

        tmp_dir = tempfile.TemporaryDirectory(dir=os.path.dirname(path))

        # Make a copy of the file to do the formatting on
        output_path = os.path.join(tmp_dir.name, os.path.basename(path))
        shutil.copy(path, output_path)

        # Apply our arguments to the formatter command
        args = {"path": output_path, **_get_args(path)}
        formatter = [[arg.format(**args) for arg in tool] for tool in formatter]

        # Format the code
        tool_text = ""
        run_args = {"stderr": STDOUT, "stdout": PIPE, "check": True}
        for c in formatter:
            # Print the command being executed
            if verbose >= 2:
                text += f"\t$ {' '.join(c)}\n"
            cmd = sp_run(c, **run_args)
            tool_text = tool_text + cmd.stdout.decode("utf-8")

        if verbose >= 1 and tool_text:
            text = text + tool_text

        if check:
            # Run the diff command
            cmd = sp_run(["colordiff", "--color=yes", "--unified", path, output_path], **run_args)
        else:
            # Check if the file has changed and if so replace the original using python
            with open(path, "rb") as f:
                with open(output_path, "rb") as g:
                    if f.read() != g.read():
                        os.rename(output_path, path)

    except CalledProcessError as e:
        text = text + e.output.decode("utf-8").strip()
        success = False

    return text, success


@run_on_docker
def register(command):
    command.help = "Format all the code in the codebase using clang-format"

    command.add_argument(
        "-v",
        "--verbose",
        action="count",
        default=0,
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
            sp_run(["git", "diff", "--name-only", "origin/main"], stdout=PIPE, check=True)
            .stdout.decode("utf-8")
            .splitlines()
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

    success = True
    with multiprocessing.Pool(multiprocessing.cpu_count()) as pool:
        for r, s in pool.imap_unordered(partial(_do_format, verbose=verbose, check=check), files):
            sys.stdout.write(r)
            success = success and s

    sys.exit(0 if success else 1)
