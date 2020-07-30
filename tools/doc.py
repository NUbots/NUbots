#!/usr/bin/python3

# clang --include-directory /home/nubots/build/shared --include-directory /home/nubots/NUbots/shared --include-directory /home/nubots/NUbots/nuclear/message/include -c -Xclang -ast-dump -fsyntax-only test.cpp
# pydoc -b tools/clang/cindex.py

import sys
import os
import re

import b
from dockerise import run_on_docker

import clang.cindex

libraryFile = "/usr/local/lib/"  # llvm-config --libdir
parseArgs = [
    "-I../build/shared",
    "-Ishared",
    "-Inuclear/message/include",
    "-I/usr/local/lib/clang/9.0.1/include",  # clang include path
    # "-Wall",
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

    print("Diagnostics")
    for diag in translationUnit.diagnostics:
        print(diag)

    cursor = translationUnit.cursor

    for node in cursor.walk_preorder():
        if str(node.location.file) == file:
            print(str(node.location))
