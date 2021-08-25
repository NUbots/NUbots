#!/usr/bin/env python3
import subprocess

from utility.dockerise import run_on_docker


@run_on_docker
def register(command):
    command.help = "Generate documentation pages from code comments using Doxygen"


@run_on_docker
def run(**kwargs):
    # Generate the documentation using the config file `.Doxyfile`
    exit(subprocess.run(["doxygen", ".Doxyfile"]).returncode)
