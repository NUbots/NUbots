from .Tree import Tree, On, Emit, Function, Reactor

import clang.cindex

import itertools

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
    root = Tree(translationUnit.diagnostics)

    for diagnostic in translationUnit.getDiagnostics():
        if diagnostic.severity >= Diagnostic.Error:
            print(diagnostic, ", ", module, "/src/", f, " will not be looked at", sep="")
            return root

    _traverseTree(translationUnit.cursor, root)

    return root


def _traverseTree(node, root):
    for child in node.get_children():
        if isFunction(child):
            root.appendFunction(makeFunction(child))
        elif isClass(child):
            if isInherited(child, "NUClear::Reactor"):
                root.appendReactor(makeReactor(child))
            else:
                _traverseTree(child, root)
        else:
            _traverseTree(child, root)


def makeFunction(node):
    function = Function(node)
    _functionTree(node, function)
    return function


def _functionTree(node, function):
    for child in node.get_children():
        if isCall(child):
            if isOnCall(child):
                function.appendOn(makeOn(child))
            elif isEmitCall(child):
                pass
            else:
                pass
        else:
            _functionTree(child, function)


def makeOn(node):
    on = On(node)
    children = node.get_children()

    try:
        dsl = ""
        for dslChild in next(next(next(next(children).get_children()).get_children()).get_children()).get_children():
            # Build up the full typename
            if dslChild.kind == clang.cindex.CursorKind.TEMPLATE_REF:
                dsl += "{}::".format(dslChild.spelling)
            elif dslChild.kind == clang.cindex.CursorKind.NAMESPACE_REF:
                dsl += "{}::".format(dslChild.spelling)
            elif dslChild.kind == clang.cindex.CursorKind.TYPE_REF:
                dsl += dslChild.type.spelling
        on.dsl = dsl
    except StopIteration as e:
        print(e)

    try:
        callbackChild = next(children)
        if callbackChild.kind == clang.cindex.CursorKind.UNEXPOSED_EXPR:
            callback = next(callbackChild.get_children())
            on.callback = makeFunction(callback)
        elif callbackChild.kind == clang.cindex.CursorKind.DECL_REF_EXPR:
            pass
            # TODO work out how to find the function referenced
            # on.callback = callbackChild.referenced
    except StopIteration as e:
        print(e)

    return on


def makeReactor(node):
    reactor = Reactor(node)
    return reactor


def isFunction(node):
    return (
        node.kind == clang.cindex.CursorKind.CXX_METHOD
        or node.kind == clang.cindex.CursorKind.CONSTRUCTOR
        or node.kind == clang.cindex.CursorKind.DESTRUCTOR
        or node.kind == clang.cindex.CursorKind.FUNCTION_DECL
        or node.kind == clang.cindex.CursorKind.CONVERSION_FUNCTION
    )


def isCall(node):
    return child.kind == clang.cindex.CursorKind.CALL_EXPR


# node must be a call
def isOnCall(node):
    return child.type.spelling == "NUClear::threading::ReactionHandle" and child.spelling == "then"


# node must be a call
def isEmitCall(node):
    return child.spelling == "emit"


def isClass(node):
    return node.kind == clang.cindex.CursorKind.CLASS_DECL


def isInherited(node, name):
    return child.kind == clang.cindex.CursorKind.CXX_BASE_SPECIFIER and child.type.spelling == name
