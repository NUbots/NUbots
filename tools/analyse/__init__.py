from .Reactor import Reactor

import clang.cindex

# TODO put in config file
libraryFile = "/usr/local/lib"  # llvm-config --libdir
clang.cindex.Config.set_library_path(libraryFile)
parseArgs = [
    "-I../build/shared",
    "-Ishared",
    "-Inuclear/message/include",
    "-I/usr/local/lib/clang/9.0.1/include",  # clang include path, clang -E -v -
    "-I/usr/local/include/eigen3",
    # "-Wall",
]

# Creates a index for parsing
def createIndex():
    return clang.cindex.Index.create()


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


# Creates a tree of reactors, on statements and emit statements
def createTree(index, file):
    translationUnit = index.parse(file, parseArgs)
    root = Reactor.Tree(translationUnit.diagnostics)

    for diagnostic in translationUnit.getDiagnostics():
        if diagnostic.severity >= Diagnostic.Error:
            print(diagnostic, ", ", module, "/src/", f, " will not be looked at", sep="")
            return root

    subTree(translationUnit.cursor)


def subTree(node):
    for child in node.get_children():
        if isFunction(child):
            pass
        elif isClass(child):
            if isInherited(child, "NUClear::Reactor"):
                pass
        else:
            subTree(child)


def functionTree(node):
    pass


def isOn(node):
    return (
        child.kind == clang.cindex.CursorKind.CALL_EXPR
        and child.type.spelling == "NUClear::threading::ReactionHandle"
        and child.spelling == "then"
    )


def isFunction(node):
    return (
        node.kind == clang.cindex.CursorKind.CXX_METHOD
        or node.kind == clang.cindex.CursorKind.CONSTRUCTOR
        or node.kind == clang.cindex.CursorKind.DESTRUCTOR
        or node.kind == clang.cindex.CursorKind.FUNCTION_DECL
        or node.kind == clang.cindex.CursorKind.CONVERSION_FUNCTION
    )


def isEmit(node):
    return child.kind == clang.cindex.CursorKind.CALL_EXPR and child.spelling == "emit"


def isClass(node):
    return node.kind == clang.cindex.CursorKind.CLASS_DECL


def isInherited(node, name):
    return child.kind == clang.cindex.CursorKind.CXX_BASE_SPECIFIER and child.type.spelling == name
