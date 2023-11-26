#
# MIT License
#
# Copyright (c) 2023 NUbots
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
import re
import subprocess
import sys

whitespace = re.compile("\s+")


def parse_requirements_map(user_tools_path):
    file_path = os.path.abspath(os.path.join(user_tools_path, "requirements.map"))
    requirements_map = {}

    with open(file_path, "r") as file:
        for line in file:
            line = line.strip()

            if len(line) == 0 or line.startswith("#"):
                continue

            import_name, package_name = whitespace.split(line.strip())
            requirements_map[import_name] = package_name

    return requirements_map


# https://regexr.com/5le57
dependency_name_regex = re.compile(r"^(.+?)(===|==|~=|!=|<=|>=|>|<)")


def parse_requirement_versions(user_tools_path):
    file_path = os.path.abspath(os.path.join(user_tools_path, "requirements.txt"))
    versions = {}

    with open(file_path, "r") as file:
        for line in file:
            line = line.strip()

            if len(line) == 0 or line.startswith("#"):
                continue

            match = dependency_name_regex.match(line)

            if match:
                versions[match.group(1)] = line

    return versions


dependency_map = None


def create_dependency_map(user_tools_path):
    requirements_map = parse_requirements_map(user_tools_path)
    versions = parse_requirement_versions(user_tools_path)

    global dependency_map
    dependency_map = {}

    for import_name, package_name in requirements_map.items():
        if package_name not in versions:
            print(
                f"error: dependency '{package_name}' in `tools/requirements.map` not found in `tools/requirements.txt`"
            )
            sys.exit(1)

        dependency_map[import_name] = {
            "import_name": import_name,
            "package_name": package_name,
            "version": versions[package_name],
        }


def find_dependency(import_name, user_tools_path):
    if dependency_map is None:
        create_dependency_map(user_tools_path)

    if import_name not in dependency_map:
        print(f"error: dependency for import '{import_name}' not found in `tools/requirements.map`")
        sys.exit(1)

    return dependency_map[import_name]


def install_dependency(*dependencies):
    subprocess.run([sys.executable, "-m", "pip", "install", *dependencies], check=True)
