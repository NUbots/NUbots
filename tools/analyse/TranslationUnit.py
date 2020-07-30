from .Class import Class
from .Reactor import Reactor

import clang.cindex

# TODO put in config
parseArgs = [
    "-I../build/shared",
    "-Ishared",
    "-Inuclear/message/include",
    "-I/usr/local/lib/clang/9.0.1/include",  # clang include path
    # "-Wall",
]


class TranslationUnit:
    def __init__(self, index, file):
        self.translationUnit = index.parse(file, parseArgs)

    # See if there are any errors in parsing
    def getDiagnostics(self):
        return self.translationUnit.diagnostics

    # The root node in the tree of the Translation Unit
    def _root(self):
        return self.translationUnit.cursor

    # Find class definitions
    def _findClassNodes(self):
        for node in self.translationUnit.cursor.walk_preorder():
            if node.kind == clang.cindex.CursorKind.CLASS_DECL:
                yield node

    # Find classes that inherit a type
    # name should be the spelling of the base class
    def _findInheritedNodes(self, name):
        for node in _findClassNodes():
            for child in node.get_children():
                if child.kind == clang.cindex.CursorKind.CXX_BASE_SPECIFIER and child.spelling == name:
                    yield node

    # Find member definitions for a class
    def _findMemberNodes(self, classNode):
        pass

    # Return objects that hold all the reactor details
    def getReactors(self):
        reactors = []
        for node in _findInheritedNodes("NUClear::Reactor"):
            reactor = Reactor(node)
            reactors.append(reactor)
        return reactors
