#!/usr/bin/env python3

try:
    import os
    import b

    from subprocess import call

    def register(command):

        # Install help
        command.help = "Format all the code in the codebase using clang-format"

    def run(**kwargs):

        # Get our relevant directories
        for d in [os.path.join(b.project_dir, p) for p in ["module", "shared", "tests"]]:
            for dirpath, dnames, fnames in os.walk(d):
                for f in fnames:
                    if f.endswith((".h", ".c", ".cc", ".cxx", ".cpp", ".hpp", ".ipp", ".proto")):
                        print("Formatting", f)
                        call(["clang-format-6.0", "-i", "-style=file", os.path.join(dirpath, f)])


except BaseException as e:
    # Capture the exception in a variable
    ex = e

    def register(command):
        # Swallow arguments
        command.add_argument("_", nargs="*")

    def run(**kwargs):
        print("Cannot run this command due to the following error")
        print(ex)
