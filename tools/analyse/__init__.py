from .Reactor import Reactor
from .TranslationUnit import TranslationUnit

import clang.cindex

# TODO put in config file
libraryFile = "/usr/local/lib"  # llvm-config --libdir
clang.cindex.Config.set_library_path(libraryFile)

# Creates a index for parsing
def createIndex():
    return clang.cindex.Index.create()


# Parse a file with the given index
def translate(index, path):
    return TranslationUnit(index, path)


# Prints out one node
def printNode(node, tab=0):
    return "  " * tab + "{} {}#{} {}:{}:{}\n".format(
        node.kind.name, node.type.spelling, node.spelling, node.location.file, node.location.line, node.location.column
    )


# Print out a kinda readable tree
def printTree(node, tab=0):
    out = ""
    out += printNode(node, tab)
    for child in node.get_children():
        out += printTree(child, tab + 1)
    return out


# Print out the tree for each unique method in a reactor
def printReactorAst(reactor):
    analyse.printTree(reactor.node)
    for _, method in reactor.getMethodsNoDuplicate().items():
        analyse.printTree(method.node)
