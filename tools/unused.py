#!/usr/bin/env python3

import os

import b
from dockerise import WrapPty, run_on_docker


# @run_on_docker
def register(command):
    # Install help
    command.help = "Creates a list of unused or commented out modules"


# @run_on_docker
def run(**kwargs):

    print("hello world")
