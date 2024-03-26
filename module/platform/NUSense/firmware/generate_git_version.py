#!/usr/bin/env python3

import subprocess
import sys

check_dir = sys.argv[1]
output_file = sys.argv[2]

# Get the latest commit hash
git_version = (
    subprocess.run(["git", "log", "-1", "--pretty=format:%H", "--", check_dir], stdout=subprocess.PIPE, check=True)
    .stdout.decode("utf-8")
    .strip()
)

if len(git_version) != 40:
    raise ValueError(f"Invalid git version: {git_version}")

# Split the git version into pairs of characters
git_version = ", ".join(f"0x{a}{b}" for a, b in zip(git_version[::2], git_version[1::2]))

output = f"""
#ifndef GIT_VERSION_HPP
#define GIT_VERSION_HPP

constexpr const uint8_t GIT_VERSION[20] = {{{git_version}}};

#endif // GIT_VERSION_HPP

"""

with open(output_file, "w") as f:
    f.write(output)
