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
        classes = []
        for node in self.translationUnit.cursor.walk_preorder():
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
        for member in self.translationUnit.cursor.walk_preorder():
            if (
                member.kind == clang.cindex.CursorKind.CXX_METHOD
                or member.kind == clang.cindex.CursorKind.CONSTRUCTOR
                or member.kind == clang.cindex.CursorKind.DESTRUCTOR
            ):
                methods.append(member)
        return methods

    def _findMethodsInClassNodes(self, node):
        methods = []
        for member in node.get_children():
            if (
                member.kind == clang.cindex.CursorKind.CXX_METHOD
                or member.kind == clang.cindex.CursorKind.CONSTRUCTOR
                or member.kind == clang.cindex.CursorKind.DESTRUCTOR
            ):
                methods.append(member)
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
