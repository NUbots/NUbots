from .Reactor import Reactor

import clang.cindex
from os import path

# TODO put in config
parseArgs = [
    "-I../build/shared",
    "-Ishared",
    "-Inuclear/message/include",
    "-I/usr/local/lib/clang/9.0.1/include",  # clang include path, clang -E -v -
    "-I/usr/local/include/eigen3",
    # "-Wall",
]


class TranslationUnit:
    def __init__(self, index, file):
        self.file = path.splitext(file)[0]
        self.translationUnit = index.parse(file, parseArgs)
        self.toLook = self._filesToLookAt()

    # See if there are any errors in parsing
    def getDiagnostics(self):
        return self.translationUnit.diagnostics

    def _filesToLookAt(self):
        out = []
        for node in self.translationUnit.cursor.get_children():
            if path.splitext(node.location.file.name)[0] == self.file:
                out.append(node)
        return out

    # Find class definitions
    def _findClassNodes(self):
        classes = []
        for look in self.toLook:
            for node in look.walk_preorder():
                if node.kind == clang.cindex.CursorKind.CLASS_DECL:
                    classes.append(node)
        return classes

    # Find classes that inherit a type
    # name should be the spelling of the base class
    def _findInheritedNodes(self, name):
        classes = []
        for node in self._findClassNodes():
            try:
                child = next(node.get_children())
                if child.kind == clang.cindex.CursorKind.CXX_BASE_SPECIFIER and child.type.spelling == name:
                    classes.append(node)
            except StopIteration:
                pass
        return classes

    # Find member definitions for a class
    def _findMethodNodes(self):
        methods = []
        for look in self.toLook:
            for node in look.walk_preorder():
                if (
                    node.kind == clang.cindex.CursorKind.CXX_METHOD
                    or node.kind == clang.cindex.CursorKind.CONSTRUCTOR
                    or node.kind == clang.cindex.CursorKind.DESTRUCTOR
                ):
                    methods.append(node)
        return methods

    def _findMethodsInClassNodes(self, node):
        methods = []
        for child in node.get_children():
            if (
                child.kind == clang.cindex.CursorKind.CXX_METHOD
                or child.kind == clang.cindex.CursorKind.CONSTRUCTOR
                or child.kind == clang.cindex.CursorKind.DESTRUCTOR
            ):
                methods.append(child)
        return methods

    def _findMethodsOutClassNodes(self, name):
        methods = []
        for node in self._findMethodNodes():
            try:
                child = next(node.get_children())
                if child.kind == clang.cindex.CursorKind.TYPE_REF and child.type.spelling == name:
                    methods.append(node)
            except StopIteration:
                pass
        return methods

    # Return objects that hold all the reactor details
    def getReactors(self):
        reactors = []
        for node in self._findInheritedNodes("NUClear::Reactor"):
            reactors.append(
                Reactor(node, self._findMethodsInClassNodes(node) + self._findMethodsOutClassNodes(node.type.spelling))
            )
        return reactors
