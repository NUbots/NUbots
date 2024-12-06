#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2021 NUbots
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

from termcolor import cprint

import b
from utility.dockerise import run_on_docker


@run_on_docker
def register(command):
    # Install help
    command.help = "Creates a list of unused or commented out modules"


@run_on_docker
def run(**kwargs):

    source_dir = b.cmake_cache[b.cmake_cache["CMAKE_PROJECT_NAME"] + "_SOURCE_DIR"]
    roles_path = os.path.join(source_dir, b.cmake_cache["NUCLEAR_ROLES_DIR"])
    modules_path = os.path.join(source_dir, b.cmake_cache["NUCLEAR_MODULE_DIR"])

    # Modules that exist in the system
    existing_modules = set()

    # Modules that are used in role files
    used_modules = set()

    # Modules that are not used by any role
    unused_modules = set()

    # Find all CMakeLists.txt in NUCLEAR_MODULE_DIR that contain a nuclear_module() call
    for folder, _, files in os.walk(modules_path):
        for module in files:
            if module == "CMakeLists.txt":
                module_path = os.path.join(folder, module)
                with open(module_path, "r") as f:
                    for line in f:
                        if "nuclear_module" in line.lower():
                            # Find nuclear_module call and make sure it is not commented out
                            if "#" not in line or line.find("#") > line.lower().find("nuclear_module"):
                                # Remove modules_path from the start of the path and
                                # CMakeLists.txt from the end of the path
                                # So module/motion/HeadController/CMakeLists.txt becomes motion/HeadController
                                path = module_path.replace(modules_path, "")
                                path = os.path.join(*(path.split(os.path.sep)[:-1]))

                                # Replace path separators with ::
                                path = path.replace(os.path.sep, "::")
                                existing_modules.add(path)
                                break

    # Find all of the used modules
    for folder, _, files in os.walk(roles_path):
        for role in files:
            if role.endswith(".role"):
                with open(os.path.join(folder, role), "r") as f:
                    for line in f:
                        if "#" in line:
                            line = line[: line.find("#")]
                        line = line.strip()
                        reg = re.findall(r"(\w+::(?:\w+(::)?)*)", line)
                        for modules in reg:
                            for module in modules:
                                if module != "" and module != "::":
                                    used_modules.add(module)

    # Find out which modules are unused
    unused_modules = existing_modules.difference(used_modules)

    cprint("Existing Modules", attrs=["bold"])
    print("\n".join(sorted(existing_modules)))
    print("\n")

    cprint("Used Modules", "green", attrs=["bold"])
    cprint("\n".join(sorted(used_modules)), "green")
    print("\n")

    cprint("Unused Modules", "red", attrs=["bold"])
    cprint("\n".join(sorted(unused_modules)), "red", attrs=["bold"])
    print("\n")
