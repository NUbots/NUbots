import os.path
import re

import clang.cindex

from .Tree import Emit, Function, On, Reactor, Tree

# TODO put in a config file
libraryPath = "/usr/local/lib"  # llvm-config --libdir
clang.cindex.Config.set_library_path(libraryPath)
parseArgs = [
    "-I../build/shared",
    "-Ishared",
    "-Inuclear/message/include",
    "-I/usr/local/lib/clang/9.0.1/include",  # clang include path, clang -E -v -
    "-I/usr/local/include/eigen3",
]

# A list of folders that could have interesting things
whitelist = [
    os.path.abspath("/usr/local/include/nuclear_bits/"),
]

# Creates a index for parsing
def createIndex():
    return clang.cindex.Index.create()


# Prints out one node
def printNode(node, tab=0):
    return "  " * tab + "{} {}#{} {}:{}:{}".format(
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
def createTree(files, folder):
    index = createIndex()
    translationUnits = {}
    diagnostics = []

    # Construct AST for each file
    i = 1
    total = len(files)
    for f in files:
        f = os.path.abspath(f)
        print("  [{}/{}] generating AST for file".format(i, total), f)
        translationUnits[f] = index.parse(f, parseArgs)
        diagnostics += list(translationUnits[f].diagnostics)
        i += 1
    # Somehow the defaults don't work
    root = Tree(diagnostics, [], {})

    # Make sure that there are no errors in the parsing
    for diagnostic in diagnostics:
        if diagnostic.severity >= clang.cindex.Diagnostic.Error:
            print(diagnostic)
            return root

    # Create an adjacency list for the topological sort
    adjList = adjacencyList()

    for tu in translationUnits.values():
        currentFile = os.path.abspath(tu.spelling)
        adjList.init_vertex(currentFile)
        for fileInclusion in tu.get_includes():
            includePath = os.path.abspath(fileInclusion.include.name)
            if os.path.commonpath([includePath, folder]) == folder:
                adjList.add_edge(currentFile, includePath)

    # Topological sort the adjacency list so we do everything in the right order
    order = topologicalSort(adjList)

    i = 1
    total = len(order)
    for fileName in order:
        print("  [{}/{}] working on file".format(i, total), fileName)
        # For the initial loop through make sure that the file the node comes from is interesting to us
        for child in translationUnits[fileName].cursor.get_children():
            childPath = os.path.abspath(child.location.file.name)

            # Check that the file is the one we want to parse
            good = os.path.commonpath([childPath, fileName]) == fileName

            if good:
                _treeNodeStuff(child, root)
        i += 1

    return root


# Just loop through the tree
def _traverseTree(node, root):
    for child in node.get_children():
        _treeNodeStuff(child, root)


# Find interesting nodes in the tree
def _treeNodeStuff(node, root):
    if isFunction(node):
        function = makeFunction(node, root)
        if function:
            root.functions[function.node.displayname] = function
    elif isClass(node):
        try:
            if isInherited(next(node.get_children()), "NUClear::Reactor"):
                root.reactors.append(makeReactor(node, root))
            else:
                _traverseTree(node, root)
        except StopIteration as e:
            _traverseTree(node, root)  # Class was a forward declare with no inheritance
    else:
        _traverseTree(node, root)


# Find information about a function
def makeFunction(node, root):
    function = Function(node, [], [])

    _functionTree(node, function, root)

    # Find if the function was a method
    # You can't declare a method before declaring a class
    try:
        child = next(node.get_children())
        if isTypeRef(child):
            for reactor in root.reactors:
                if child.type.spelling == reactor.node.type.spelling:
                    reactor.methods.append(function)
    except StopIteration as e:
        pass  # Function was a forward declared and not a method

    # Check that the function is interesting
    if function.emits or function.ons or not function.calls == []:
        return function


# loop through the function finding interesting information
def _functionTree(node, function, root):
    for child in node.get_children():
        if isCall(child):
            if isOnCall(child):
                function.ons.append(makeOn(child, root))
            elif isEmitCall(child):
                function.emits.append(makeEmit(child))
            else:
                # If the function is calling another function, if the other function is in the whitelist, give it a look
                definition = child.get_definition()
                if definition and not root.functions.get(definition.displayname):
                    for fileName in whitelist:
                        if os.path.commonpath([os.path.abspath(definition.location.file.name), fileName]) == fileName:
                            calledFunction = makeFunction(definition, root)
                            if calledFunction:
                                root.functions[calledFunction.node.displayname] = calledFunction
                                function.calls.append(calledFunction)
                                function.calls.extend(calledFunction.calls)
                            break

        else:
            _functionTree(child, function, root)


# Find information about an on node
def makeOn(node, root):
    on = On(node)
    children = node.get_children()

    # Find the dsl type by looping through the expected part of the tree
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

    # Find the callback for the function
    try:
        for callbackChild in children:
            if callbackChild.kind == clang.cindex.CursorKind.UNEXPOSED_EXPR:  # lambda
                callback = next(callbackChild.get_children())
                function = makeFunction(callback, root)
                if function:
                    on.callback = function
            elif callbackChild.kind == clang.cindex.CursorKind.DECL_REF_EXPR:  # reference to predefined function
                for function in root.functions.values():
                    if function.node == callbackChild.referenced:
                        on.callback = function
    except StopIteration as e:
        print("On callback find StopIter:", e)

    return on


# Find information about an emit node
def makeEmit(node):
    emit = Emit(node)

    children = node.get_children()

    # Work out the scope of the node
    emit.scope = "Local"
    try:
        memberRefExpr = next(children)
        for part in memberRefExpr.get_children():
            if part.kind == clang.cindex.CursorKind.TEMPLATE_REF:
                emit.scope = part.spelling
    except StopIteration as e:
        print("Emit scope StopIter:", e)

    # Work out the type that is being emitted
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
    except StopIteration as e:
        print("Emit type StopIter:", e)

    return emit


# Information about a reactor
def makeReactor(node, root):
    reactor = Reactor(node, [])

    # Remove forward declared reactors
    shift = 0
    last = len(root.reactors)
    for i in range(last):
        if root.reactors[i - shift].node.type.spelling == node.type.spelling:
            reactor.methods.extend(root.reactors[i - shift].methods)
            del root.reactors[i - shift]
            shift += 1

    # Find the methods in the reactor
    for child in node.get_children():
        if isCall(child):
            function = makeFunction(child)
            if function:
                reactor.methods.append(function)
                root.functions[function.node.displayname] = function

    return reactor


# Checks all nodes types that could be functions
def isFunction(node):
    return (
        node.kind == clang.cindex.CursorKind.CXX_METHOD
        or node.kind == clang.cindex.CursorKind.CONSTRUCTOR
        or node.kind == clang.cindex.CursorKind.DESTRUCTOR
        or node.kind == clang.cindex.CursorKind.FUNCTION_DECL
        or node.kind == clang.cindex.CursorKind.CONVERSION_FUNCTION
    )


# Checks that a node is a type ref, the first child of a function that is a method will be a type ref.
# node the first child of a function outside a class
def isTypeRef(node):
    return node.kind == clang.cindex.CursorKind.TYPE_REF


# Is a function call
def isCall(node):
    return node.kind == clang.cindex.CursorKind.CALL_EXPR


# Check that a call is an on node
# Node must be a call
def isOnCall(node):
    return node.type.spelling == "NUClear::threading::ReactionHandle" and node.spelling == "then"


# Check that a call is an emit statement
# node must be a call
def isEmitCall(node):
    return node.spelling == "emit"


# Check that the node is a class decleration
def isClass(node):
    return node.kind == clang.cindex.CursorKind.CLASS_DECL


# The first child of a class decleration will have information about the parent if it is an child
def isInherited(node, name):
    return node.kind == clang.cindex.CursorKind.CXX_BASE_SPECIFIER and node.type.spelling == name


class adjacencyList:
    def __init__(self):
        self.innerList = {}
        self.size = 0

    def init_vertex(self, u):
        if not self.innerList.get(u):
            self.innerList[u] = []

    def add_edge(self, u, v):
        if not self.innerList.get(u):
            self.innerList[u] = []
        self.innerList[u].append(v)

    def get(self, key):
        return self.innerList.get(key)

    def __getitem__(self, key):
        return self.innerList[key]

    def __len__(self):
        return self.size

    def __iter__(self):
        return iter(self.innerList)

    def __repr__(self):
        out = ""
        for u in self:
            out += repr(u) + " -> " + repr([v for v in self.innerList[u]]) + ","
        return out


# Topological sorts a given adjacency list
def topologicalSort(adj):
    tSorted = []
    visit = {}
    for u in adj:
        visit[u] = False

    for u in adj:
        if not visit[u]:
            _topologicalSortRecurs(adj, u, tSorted, visit)

    return tSorted


def _topologicalSortRecurs(adj, start, tSorted, visit):
    visit[start] = True
    trav = adj[start]
    for travIndex in range(len(trav)):
        v = trav[travIndex]
        if not visit[v]:
            _topologicalSortRecurs(adj, v, tSorted, visit)

    tSorted.append(start)
