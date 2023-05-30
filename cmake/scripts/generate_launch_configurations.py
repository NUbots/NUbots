#!/usr/bin/env python3

import argparse
import json
import os


def generate(project_source_dir, project_binary_dir, use_asan, roles):
    configurations = []
    for role in roles:
        if "-" in role:
            role_path = role.split("-")
            role_name = " ".join([r.title() for r in role_path[-1].split("_")]) + ": " + os.path.join(*role_path[:-1])
            role_binary = os.path.join(project_binary_dir, "bin", *role_path)
        else:
            role_name = " ".join([r.title() for r in role.split("_")])
            role_binary = os.path.join(project_binary_dir, "bin", role)
        configurations.append(
            {
                "name": role_name,
                "type": "cppdbg",
                "request": "launch",
                "program": role_binary,
                "args": [],
                "additionalSOLibSearchPath": f"{project_binary_dir}/bin/lib;{project_binary_dir}/bin",
                "stopAtEntry": True,
                "cwd": project_binary_dir,
                "environment": [{"name": "GUILE_AUTO_COMPILE", "value": "0"}],
                "externalConsole": False,
                "MIMode": "gdb",
                "setupCommands": [
                    {
                        "description": "Enable pretty-printing for gdb",
                        "text": "-enable-pretty-printing",
                        "ignoreFailures": True,
                    },
                    {
                        "description": "Set Disassembly Flavor to Intel",
                        "text": "-gdb-set disassembly-flavor intel",
                        "ignoreFailures": True,
                    },
                    {
                        "description": "Turn on pending breakpoints",
                        "text": "-ex set breakpoint pending on",
                        "ignoreFailures": True,
                    },
                    {
                        "description": "Set breakpoint to catch ASan error reporting",
                        "text": "-ex br __asan::ReportGenericError",
                        "ignoreFailures": True,
                    },
                    {
                        "description": "Disable debuginfod",
                        "text": "-ex set debuginfod enabled off",
                        "ignoreFailures": True,
                    },
                ],
            }
        )

        if use_asan:
            configurations[-1]["environment"].append(
                {"name": "ASAN_OPTIONS", "value": f"log_path={project_source_dir}/log"}
            )

    return {"versions": "0.2.0", "configurations": configurations}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate launch configurations for debugging roles")
    parser.add_argument(
        "--source-directory", "-s", default=os.path.join("", "home", "NUbots", "NUbots"), help="Location of source code"
    )
    parser.add_argument(
        "--binary-directory",
        "-b",
        default=os.path.join("", "home", "NUbots", "build"),
        help="Location of build directory",
    )
    parser.add_argument(
        "--launch-file",
        "-l",
        default=os.path.join("", "home", "NUbots", "NUbots", ".vscode", "launch.json"),
        help="Location of build directory",
    )
    parser.add_argument(
        "--use-asan",
        "-a",
        help="Include ASAN_OPTIONS environment variable in launch configurations",
    )
    parser.add_argument("roles", nargs="*", help="Roles to generate launch configurations for")
    args = parser.parse_args()

    roles = []
    for role in args.roles:
        roles.extend(role.split(" "))

    with open(args.launch_file, "w") as f:
        json.dump(
            generate(
                project_source_dir=args.source_directory,
                project_binary_dir=args.binary_directory,
                use_asan=args.use_asan.upper() == "ON",
                roles=roles,
            ),
            f,
            ensure_ascii=True,
            indent=2,
        )
