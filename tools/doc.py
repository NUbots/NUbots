#!/usr/bin/python3

# clang --include-directory /home/nubots/build/shared --include-directory /home/nubots/NUbots/shared --include-directory /home/nubots/NUbots/nuclear/message/include -c -Xclang -ast-dump -fsyntax-only test.cpp

import sys
import os
import re

import b
from dockerise import run_on_docker

import clang.cindex

libraryFile = "/usr/local/lib/"  # llvm-config --libdir
parseArgs = [
    "--include-directory /home/nubots/build/shared",
    "--include-directory /home/nubots/NUbots/shared",
    "--include-directory /home/nubots/NUbots/nuclear/message/include",
    "-c",
]


@run_on_docker
def register(command):
    command.help = "documentation generation?"

    command.add_argument("file", help="The file to do")


@run_on_docker
def run(file, **kwargs):

    clang.cindex.Config.set_library_path(libraryFile)

    index = clang.cindex.Index.create()

    translationUnit = index.parse(file, parseArgs)

    cursor = translationUnit.cursor

    for node in cursor.walk_preorder():
        if str(node.location.file) == file:
            print(node.location)
