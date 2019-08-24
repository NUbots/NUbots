#!/usr/bin/env python3

from nudocker import run_on_docker
import os


@run_on_docker
def register(command):
    # Install help
    command.help = "Open an interactive shell in a docker container"


@run_on_docker
def run(**kwargs):
    print("I suppose now I should run a shell... like /bin/bash or something but make sure it's interactive")
