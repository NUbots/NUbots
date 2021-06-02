#! /usr/bin/env python3

import os
import textwrap
from glob import glob

import b
from utility.dockerise import run_on_docker


@run_on_docker
def register(command):
    # Install help
    command.help = "Find all unused modules"

    command.add_argument(
        "--export-role",
        dest="export",
        action="store_true",
        default=False,
        help="Create a role containing all unused modules",
    )


@run_on_docker
def run(export, **kwargs):
    base = b.project_dir
    module_base = os.path.join(base, b.cmake_cache["NUCLEAR_MODULE_DIR"])
    role_base = os.path.join(base, b.cmake_cache["NUCLEAR_ROLES_DIR"])

    # Find all module folders that contain a CMakeLists.txt file
    potential_modules = glob(os.path.join(module_base, "**", "CMakeLists.txt"), recursive=True)

    # Filter potential modules down to only those whose CMakeLists.txt contains 'nuclear_module'
    modules = []
    for module in potential_modules:
        with open(module, "r") as f:
            contents = f.read()
        if "nuclear_module" in contents.lower():
            modules.append(os.path.relpath(os.path.dirname(module), module_base).replace(os.sep, "::"))

    # Find all of the role files
    role_files = glob(os.path.join(role_base, "*.role"), recursive=True)

    # Extract from the role files all of the used modules
    used_modules = []
    for role in role_files:
        with open(role, "r") as f:
            for module in f:
                # Exclude lines containing 'nuclear_role', '(' and ')'
                if "nuclear_role" not in module and "(" not in module and ")" not in module:
                    # Exclude commented lines
                    if len(module[: module.find("#")].strip()) > 0:
                        used_modules.append(module[: module.find("#")].strip())

    # Remove used modules from the list of modules
    modules = set(modules) - set(used_modules)

    # Show the results
    if len(modules) > 0:
        if export:
            role_template = textwrap.dedent(
                """\
                nuclear_role(
                  {modules}
                )
                """
            )
            with open(os.path.join(role_base, "unused.role"), "w") as f:
                f.write(role_template.format(modules="\n  ".join(modules)))
        else:
            role_template = textwrap.dedent(
                """\
                Unused modules:
                  {modules}
                """
            )
            print(role_template.format(modules="\n  ".join(modules)))
    else:
        print("No unused modules")
