#!/usr/bin/env python3

from dockerise import run_on_docker
import os
import pty


@run_on_docker
def register(command):
    # Install help
    command.help = "Open an interactive shell in a docker container"


@run_on_docker
def run(**kwargs):
    # Run bash
    pty.spawn("/bin/bash")
