from .Tree import Tree, On, Emit, Function, Reactor

import clang.cindex

import re
import itertools

# TODO put in a config file
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


root = None

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
            try:
                if isInherited(next(child.get_children()), "NUClear::Reactor"):
                    root.appendReactor(makeReactor(child))
                    continue
            except StopIteration as e:
                print(e)
            _traverseTree(child, root)
        else:
            _traverseTree(child, root)


def makeFunction(node):
    function = Function(node)

    _functionTree(node, function)

    # You can't declare a method before declaring a class
    try:
        child = next(node.get_children())
        if isMethod(child):
            for reactor in root.reactors:
                if child.type.spelling == reactor.node.type.spelling:
                    reactor.appendFunction(function)
    except StopIteration as e:
        print(e)

    return function


def _functionTree(node, function):
    for child in node.get_children():
        if isCall(child):
            if isOnCall(child):
                function.appendOn(makeOn(child))
            elif isEmitCall(child):
                function.appendEmit(makeEmit(child))
            else:
                function.appendCall(child)
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
            for function in root.functions:
                if function.node == callbackChild.referenced:
                    on.callback = function
    except StopIteration as e:
        print(e)

    return on


def makeEmit(node):
    emit = Emit(node)

    children = node.get_children()

    emit.scope = "Local"
    try:
        memberRefExpr = next(children)
        for part in memberRefExpr.get_children():
            if part.kind == clang.cindex.CursorKind.TEMPLATE_REF:
                emit.scop = part.spelling
    except StopIteration:
        pass

    try:
        expr = next(children)
        if expr.kind == clang.cindex.CursorKind.UNEXPOSED_EXPR:  # The parameter is constructed in the emit statement
            regexed = re.findall(Emit.makeUniqueRegex, expr.type.spelling)
            if regexed:
                return regexed[0]
            else:
                return re.findall(Emit.nusightDataRegex, expr.type.spelling)[0]
        elif (
            expr.kind == clang.cindex.CursorKind.DECL_REF_EXPR
        ):  # The parameter is constructed outside the emit statement
            return re.findall(Emit.existingUniqueRegex, expr.type.spelling)[0]
        elif expr.kind == clang.cindex.CursorKind.MEMBER_REF_EXPR:  # the parameter is a member of a class
            return expr.type.spelling
    except StopIteration:
        pass

    return emit


def makeReactor(node):
    reactor = Reactor(node)

    # Remove forward declared reactors
    shift = 0
    for i in range(root.reactors.length):
        if root.reactors[i - shift].node.type.name == node.type.name:
            reactor.addMethods(root.reactors[i - shift].methods)
            del root.rectors[i - shift]
            shift += 1

    for child in node.get_children():
        if isCall(child):
            reactor.appendMethod(makeFunction(child))

    return reactor


def isFunction(node):
    return (
        node.kind == clang.cindex.CursorKind.CXX_METHOD
        or node.kind == clang.cindex.CursorKind.CONSTRUCTOR
        or node.kind == clang.cindex.CursorKind.DESTRUCTOR
        or node.kind == clang.cindex.CursorKind.FUNCTION_DECL
        or node.kind == clang.cindex.CursorKind.CONVERSION_FUNCTION
    )


# node must be a function outside a class
def isMethod(node):
    return child.kind == clang.cindex.CursorKind.TYPE_REF


def isCall(node):
    return node.kind == clang.cindex.CursorKind.CALL_EXPR


# node must be a call
def isOnCall(node):
    return node.type.spelling == "NUClear::threading::ReactionHandle" and node.spelling == "then"


# node must be a call
def isEmitCall(node):
    return node.spelling == "emit"


def isClass(node):
    return node.kind == clang.cindex.CursorKind.CLASS_DECL


def isInherited(node, name):
    return node.kind == clang.cindex.CursorKind.CXX_BASE_SPECIFIER and node.type.spelling == name
