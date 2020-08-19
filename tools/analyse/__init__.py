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


# Creates a tree of reactors, on statements and emit statements
def createTree(index, f):
    translationUnit = index.parse(f, parseArgs)
    root = Tree(translationUnit.diagnostics)

    for diagnostic in translationUnit.diagnostics:
        if diagnostic.severity >= Diagnostic.Error:
            print(diagnostic)
            return root

    _traverseTree(translationUnit.cursor, root)

    return root


def _traverseTree(node, root):
    for child in node.get_children():
        if isFunction(child):
            root.functions.append(makeFunction(child, root))
        elif isClass(child):
            try:
                if isInherited(next(child.get_children()), "NUClear::Reactor"):
                    root.reactors.append(makeReactor(child, root))
                    continue
            except StopIteration as e:
                pass  # Class was a forward declare with no inheritance
            _traverseTree(child, root)
        else:
            _traverseTree(child, root)


def makeFunction(node, root):
    function = Function(node)

    _functionTree(node, function)

    # You can't declare a method before declaring a class
    try:
        child = next(node.get_children())
        if isMethod(child):
            for reactor in root.reactors:
                if child.type.spelling == reactor.node.type.spelling:
                    reactor.methods.append(function)
    except StopIteration as e:
        pass  # Function was a forward declared and not a method

    return function


def _functionTree(node, function):
    for child in node.get_children():
        if isCall(child):
            if isOnCall(child):
                function.ons.append(makeOn(child))
            elif isEmitCall(child):
                function.emits.append(makeEmit(child))
            else:
                function.calls.append(child)
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
        print("On DSL find StopIter:", e)

    try:
        callbackChild = next(children)
        if callbackChild.kind == clang.cindex.CursorKind.UNEXPOSED_EXPR:
            callback = next(callbackChild.get_children())
            on.callback = makeFunction(callback, root)
        elif callbackChild.kind == clang.cindex.CursorKind.DECL_REF_EXPR:
            for function in root.functions:
                if function.node == callbackChild.referenced:
                    on.callback = function
    except StopIteration as e:
        print("On callback find StopIter:", e)

    return on


def makeEmit(node):
    emit = Emit(node)

    children = node.get_children()

    emit.scope = "Local"
    try:
        memberRefExpr = next(children)
        for part in memberRefExpr.get_children():
            if part.kind == clang.cindex.CursorKind.TEMPLATE_REF:
                emit.scope = part.spelling
    except StopIteration:
        print("Emit scope StopIter:", e)

    try:
        expr = next(children)
        if expr.kind == clang.cindex.CursorKind.UNEXPOSED_EXPR:  # The parameter is constructed in the emit statement
            regexed = re.findall(Emit.makeUniqueRegex, expr.type.spelling)
            if regexed:
                emit.type = regexed[0]
            else:
                emit.type = re.findall(Emit.nusightDataRegex, expr.type.spelling)[0]
        elif (
            expr.kind == clang.cindex.CursorKind.DECL_REF_EXPR
        ):  # The parameter is constructed outside the emit statement
            regexed = re.findall(Emit.existingUniqueRegex, expr.type.spelling)
            if regexed:
                emit.type = regexed[0]
            else:  # The parameter is not a unique pointer
                emit.type = expr.type.spelling
        elif expr.kind == clang.cindex.CursorKind.MEMBER_REF_EXPR:  # the parameter is a member of a class
            emit.type = expr.type.spelling
    except StopIteration:
        print("Emit type StopIter:", e)

    return emit


def makeReactor(node, root):
    reactor = Reactor(node)

    # Remove forward declared reactors
    shift = 0
    last = len(root.reactors)
    for i in range(last):
        if root.reactors[i - shift].node.type.spelling == node.type.spelling:
            reactor.methods.extend(root.reactors[i - shift].methods)
            del root.rectors[i - shift]
            shift += 1

    for child in node.get_children():
        if isCall(child):
            function = makeFunction(child)
            reactor.methods.append(function)
            root.functions.append(function)

    return reactor


def isFunction(node):
    return (
        node.kind == clang.cindex.CursorKind.CXX_METHOD
        or node.kind == clang.cindex.CursorKind.CONSTRUCTOR
        or node.kind == clang.cindex.CursorKind.DESTRUCTOR
        or node.kind == clang.cindex.CursorKind.FUNCTION_DECL
        or node.kind == clang.cindex.CursorKind.CONVERSION_FUNCTION
    )


# node the first child of a function outside a class
def isMethod(node):
    return node.kind == clang.cindex.CursorKind.TYPE_REF


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
