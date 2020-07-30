#!/usr/bin/python3

# clang --include-directory /home/nubots/build/shared --include-directory /home/nubots/NUbots/shared --include-directory /home/nubots/NUbots/nuclear/message/include -c -Xclang -ast-dump -fsyntax-only test.cpp
# pydoc -b tools/clang/cindex.py

import sys
import os
import re

import b
from dockerise import run_on_docker
import analyse


@run_on_docker
def register(command):
    command.help = "documentation generation?"

    command.add_argument("path", help="The file to do")


@run_on_docker
def run(path, **kwargs):
    index = analyse.createIndex()
    tu = analyse.translate(index, path)
    reactors = tu.getReactors()
    for reactor in reactors:
        print(reactor)
