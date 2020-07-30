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

# Print out a kinda readable tree
def printOutTree(node, tab=0):
    print(
        "  " * tab
        + "{} {} {} {}:{}".format(
            node.kind.name, node.type.spelling, node.spelling, node.location.line, node.location.column
        )
    )
    for child in node.get_children():
        printOut(child, tab + 1)


# Find emit statement
def findEmit(node):
    for child in node.walk_preorder():
        if child.kind == clang.cindex.CursorKind.CALL_EXPR and child.spelling == "emit":
            yield child


# Find reactor inheritance
def findReactor(root):
    for node in root.walk_preorder():
        if node.kind == clang.cindex.CursorKind.CLASS_DECL:
            for child in node.get_children():
                if child.kind == clang.cindex.CursorKind.CXX_BASE_SPECIFIER and child.spelling == "NUClear::Reactor":
                    yield node


@run_on_docker
def register(command):
    command.help = "documentation generation?"

    command.add_argument("file", help="The file to do")


@run_on_docker
def run(file, **kwargs):

    clang.cindex.Config.set_library_path(libraryFile)

    index = clang.cindex.Index.create()

    translationUnit = index.parse(file, parseArgs)

    diagnostics = translationUnit.diagnostics
    if len(diagnostics) > 0:
        print("Diagnostics")
        for diag in translationUnit.diagnostics:
            print(diag)
            return

    cursor = translationUnit.cursor

    # Should make topLevel[0] the h file and topLevel[1] the cpp file
    # topLevel = []
    # for node in cursor.get_children():
    #    if (
    #        node.location.file
    #        and os.path.splitext(os.path.basename(node.location.file.name))[0]
    #        == os.path.splitext(os.path.basename(file))[0]
    #    ):
    #        topLevel.append(node)

    for thing in findReactor(cursor):
        print(thing.type.spelling)
