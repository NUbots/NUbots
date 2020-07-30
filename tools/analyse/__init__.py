from .Reactor import Reactor
from .TranslationUnit import TranslationUnit

import clang.cindex

# TODO put in cofig file
libraryFile = "/usr/local/lib"  # llvm-config --libdir
clang.cindex.Config.set_library_path(libraryFile)


def createIndex():
    return clang.cindex.Index.create()


def translate(index, path):
    return TranslationUnit(index, path)


# Prints out one node
def printNode(node, tab=0):
    print(
        "  " * tab
        + "{} {} {} {}:{}".format(
            node.kind.name, node.type.spelling, node.spelling, node.location.line, node.location.column
        )
    )


# Print out a kinda readable tree
def printTree(node, tab=0):
    printNode(node, tab)
    for child in node.get_children():
        printTree(child, tab + 1)
