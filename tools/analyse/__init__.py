import os.path
import re
import sys

import clang.cindex

from .Tree import Alias, Emit, Function, On, Reactor, Tree

# The path to the folder that contains libclang
library_path = "/usr/local/lib"  # llvm-config --libdir
clang.cindex.Config.set_library_path(library_path)

# The arguments to pass to clang
parse_args = [
    "-I../build/shared",
    "-Ishared",
    "-Inuclear/message/include",
    "-I/usr/local/lib/clang/9.0.1/include",  # clang include path, clang -E -v -
    "-I/usr/local/include/eigen3",
    "-I/usr/local/include/aravis-0.8/",
    "-I/usr/local/include/glib-2.0/",
    "--std=c++17",
]

# A list of folders that could have interesting things
whitelist = [
    os.path.abspath("/usr/local/include/nuclear_bits/"),
]

# Creates a index for parsing
def create_index():
    return clang.cindex.Index.create()


# Prints out one node
def print_node(node, tab=0):
    return "  " * tab + "{} {}#{} {}:{}:{}".format(
        node.kind.name, node.type.spelling, node.spelling, node.location.file, node.location.line, node.location.column
    )


# Print out a kinda readable tree
def print_tree(node, tab=0):
    out = ""
    out += print_node(node, tab) + "\n"
    for child in node.get_children():
        out += print_tree(child, tab + 1)
    return out


# Creates a tree of reactors, on statements and emit statements
def create_tree(files, folder):
    index = create_index()
    translation_units = {}
    diagnostics = []

    # Construct AST for each file
    i = 1
    total = len(files)
    for f in files:
        f = os.path.abspath(f)
        print("  [{}/{}] generating AST for file".format(i, total), f)
        translation_units[f] = index.parse(f, parse_args.copy() + ["-I" + folder + "/src"])
        diagnostics += list(translation_units[f].diagnostics)
        i += 1

    root = Tree(diagnostics, [], {})

    # Make sure that there are no errors in the parsing
    for diagnostic in diagnostics:
        if diagnostic.severity >= clang.cindex.Diagnostic.Error:
            print(diagnostic, file=sys.stderr)
            return root

    # Create an adjacency list for the topological sort
    adj_list = AdjacencyList()

    # Add edges to the adjacency list
    for tu in translation_units.values():
        current_file = os.path.abspath(tu.spelling)
        adj_list.init_vertex(current_file)
        for file_inclusion in tu.get_includes():
            include_path = os.path.abspath(file_inclusion.include.name)
            if os.path.commonpath([include_path, folder]) == folder:
                adj_list.add_edge(current_file, include_path)

    # Topological sort the adjacency list so we do everything in the right order
    order = topological_sort(adj_list)

    # Go through the files, pulling out relevant information
    i = 1
    total = len(order)
    for file_name in order:
        print("  [{}/{}] working on file".format(i, total), file_name)
        # For the initial loop through make sure that the file the node comes from is interesting to us
        for child in translation_units[file_name].cursor.get_children():
            child_path = os.path.abspath(child.location.file.name)

            if os.path.commonpath([child_path, file_name]) == file_name:
                _tree_node_stuff(child, root)
        i += 1

    return root


# Just loop through the children of the current node
def _traverse_tree(node, root):
    for child in node.get_children():
        root.alias_stack.append([])
        _tree_node_stuff(child, root)
        root.alias_stack.pop()


# Find interesting nodes in the tree
def _tree_node_stuff(node, root):
    # Work out where the start of the scope is so we know where our using statements are meant to be
    scope_start = False
    if is_scope_start(node):
        root.alias_stack.append([])
        scope_start = True

    if is_function(node):
        function = make_function(node, root)
        if function:
            root.functions[function.node.displayname] = function
    elif is_class(node):
        try:
            if is_inherited(next(node.get_children()), "NUClear::Reactor"):
                root.reactors.append(make_reactor(node, root))
            else:
                _traverse_tree(node, root)
        except StopIteration as e:
            _traverse_tree(node, root)  # Class was a forward declare with no inheritance
    elif is_using(node):
        root.alias_stack[-1].append(parse_alias(node))
    else:
        _traverse_tree(node, root)

    # If the node was the start of a scope, only it's children will be in that scope
    if scope_start:
        root.alias_stack.pop()


# Find information about a function
def make_function(node, root):
    function = Function(node)

    _function_tree(node, function, root)

    # Find if the function was a method
    # You can't declare a method before declaring a class
    try:
        child = next(node.get_children())
        if is_type_ref(child):
            for reactor in root.reactors:
                if child.type.spelling == reactor.node.type.spelling:
                    reactor.methods.append(function)
    except StopIteration as e:
        pass  # Function was a forward declared and not a method

    # Check that the function is interesting
    if (not function.emits == []) or (not function.ons == []) or (not function.calls == []):
        return function


# loop through the function finding interesting information
def _function_tree(node, function, root):
    for child in node.get_children():
        if is_call(child):
            if is_on_call(child):
                function.ons.append(make_on(child, root))
            elif is_emit_call(child):
                function.emits.append(make_emit(child, root))
            else:
                # If the function is calling another function, if the other function is in the whitelist
                # or if the other function is already parsed, give it a look
                definition = child.get_definition()
                if definition:
                    called_function = root.functions.get(definition.displayname)
                    if not called_function:
                        # A new function that we have not seen yet
                        for file_name in whitelist:
                            if (
                                os.path.commonpath([os.path.abspath(definition.location.file.name), file_name])
                                == file_name
                            ):
                                called_function = make_function(definition, root)
                                if called_function:
                                    root.functions[called_function.node.displayname] = called_function
                                    function.calls.append(called_function)
                                    function.calls.extend(called_function.calls)
                                break
                    else:
                        # A function we have already seen
                        function.calls.append(called_function)
                        function.calls.extend(called_function.calls)

        else:
            _function_tree(child, function, root)


# Find information about an on node
def make_on(node, root):
    on = On(node)
    children = node.get_children()

    # Find the dsl type by looping through the expected part of the tree
    try:
        dsl = ""
        for dsl_child in next(next(next(next(children).get_children()).get_children()).get_children()).get_children():
            # Build up the full typename
            if dsl_child.kind == clang.cindex.CursorKind.TEMPLATE_REF:
                dsl += "{}::".format(dsl_child.spelling)
            elif dsl_child.kind == clang.cindex.CursorKind.NAMESPACE_REF:
                dsl += "{}::".format(dsl_child.spelling)
            elif dsl_child.kind == clang.cindex.CursorKind.TYPE_REF:
                dsl += dsl_child.type.spelling
        on.dsl = dsl
    except StopIteration as e:
        print("On DSL find StopIter:", e)

    # If the type was aliased, instead store the original type
    if on.dsl:
        for l in root.alias_stack:
            for alias in l:
                if on.dsl == alias.aliased:
                    on.dsl = alias.original

    # Find the callback for the function
    try:
        for callback_child in children:
            if callback_child.kind == clang.cindex.CursorKind.UNEXPOSED_EXPR:  # lambda
                callback = next(callback_child.get_children())
                function = make_function(callback, root)
                if function:
                    on.callback = function
            elif callback_child.kind == clang.cindex.CursorKind.DECL_REF_EXPR:  # reference to predefined function
                for function in root.functions.values():
                    if function.node == callback_child.referenced:
                        on.callback = function
    except StopIteration as e:
        print("On callback find StopIter:", e)

    return on


# Find information about an emit node
def make_emit(node, root):
    emit = Emit(node)

    children = node.get_children()

    # Work out the scope of the node
    emit.scope = "Local"
    try:
        member_ref_expr = next(children)
        for part in member_ref_expr.get_children():
            if part.kind == clang.cindex.CursorKind.TEMPLATE_REF:
                emit.scope = part.spelling
    except StopIteration as e:
        print("Emit scope StopIter:", e)

    # Work out the type that is being emitted
    try:
        expr = next(children)
        if expr.kind == clang.cindex.CursorKind.UNEXPOSED_EXPR:  # The parameter is constructed in the emit statement
            regexed = re.findall(Emit.make_unique_regex, expr.type.spelling)
            if regexed:
                emit.type = regexed[0]
            else:
                emit.type = re.findall(Emit.nusight_data_regex, expr.type.spelling)[0]
        elif (
            expr.kind == clang.cindex.CursorKind.DECL_REF_EXPR
        ):  # The parameter is constructed outside the emit statement
            regexed = re.findall(Emit.existing_unique_regex, expr.type.spelling)
            if regexed:
                emit.type = regexed[0]
            else:  # The parameter is not a unique pointer
                emit.type = expr.type.spelling
        elif expr.kind == clang.cindex.CursorKind.MEMBER_REF_EXPR:  # the parameter is a member of a class
            emit.type = expr.type.spelling
    except StopIteration as e:
        print("Emit type StopIter:", e)

    # If the type was aliased, instead store the original type
    if emit.type:
        for l in root.alias_stack:
            for alias in l:
                if emit.type == alias.aliased:
                    emit.type = alias.original

    return emit


# Information about a reactor
def make_reactor(node, root):
    reactor = Reactor(node, [])

    # Find if this reactor is forward declared
    for candidate_reactor in root.reactors:
        if candidate_reactor.node.type.spelling == node.type.spelling:
            reactor = candidate_reactor
            reactor.node = node

    # Find the methods in the reactor
    for child in node.get_children():
        if is_call(child):
            function = make_function(child, root)
            if function:
                reactor.methods.append(function)
                root.functions[function.node.displayname] = function

    return reactor


# Works out type aliases from using statements
def parse_alias(node):
    original = ""
    aliased = ""

    if node.kind == clang.cindex.CursorKind.USING_DECLARATION:
        # A using declaration. i.e. `using namespace::typename;`
        aliased = node.spelling
        for child in node.get_children():
            original += child.spelling + "::"
        if original:
            original = original[:-2]
    elif node.kind == clang.cindex.CursorKind.TYPE_ALIAS_DECL:
        # A type alias declaration. i.e. `using typename = namespace::typename`
        aliased = node.type.spelling
        template_level = 0
        for child in node.get_children():
            if child.kind == clang.cindex.CursorKind.NAMESPACE_REF:
                original += child.spelling + "::"
            if child.kind == clang.cindex.CursorKind.TEMPLATE_REF:
                original += child.spelling + "<"
                template_level += 1
            if child.kind == clang.cindex.CursorKind.TYPE_REF:
                if template_level == 0:
                    # In this case there was no templated types
                    original = child.type.spelling
                else:
                    original += child.type.spelling + "> "
                    template_level -= 1
        if original == "" or original.endswith("::") or original.endswith("<"):
            print("Did not find TYPE_REF under TYPE_ALIAS_DECL")  # TODO
        for _ in range(template_level):
            original += ">"
    elif node.kind == clang.cindex.CursorKind.NAMESPACE_ALIAS:
        print("A namespace alias was used, this is currently a TODO")

    return Alias(node, original, aliased)


# Checks all nodes types that could be functions
def is_function(node):
    return (
        node.kind == clang.cindex.CursorKind.CXX_METHOD
        or node.kind == clang.cindex.CursorKind.CONSTRUCTOR
        or node.kind == clang.cindex.CursorKind.DESTRUCTOR
        or node.kind == clang.cindex.CursorKind.FUNCTION_DECL
        or node.kind == clang.cindex.CursorKind.CONVERSION_FUNCTION
    )


# Checks that a node is a using statement
def is_using(node):
    return (
        node.kind == clang.cindex.CursorKind.USING_DECLARATION
        or node.kind == clang.cindex.CursorKind.TYPE_ALIAS_DECL
        or node.kind == clang.cindex.CursorKind.NAMESPACE_ALIAS
    )


def is_scope_start(node):
    return (
        is_function(node)
        or node.kind == clang.cindex.CursorKind.STRUCT_DECL
        or node.kind == clang.cindex.CursorKind.UNION_DECL
        or node.kind == clang.cindex.CursorKind.CLASS_DECL
        or node.kind == clang.cindex.CursorKind.ENUM_DECL
        or node.kind == clang.cindex.CursorKind.NAMESPACE
        or node.kind == clang.cindex.CursorKind.LINKAGE_SPEC
        or node.kind == clang.cindex.CursorKind.COMPOUND_STMT
        or node.kind == clang.cindex.CursorKind.IF_STMT
        or node.kind == clang.cindex.CursorKind.SWITCH_STMT
        or node.kind == clang.cindex.CursorKind.WHILE_STMT
        or node.kind == clang.cindex.CursorKind.DO_STMT
        or node.kind == clang.cindex.CursorKind.FOR_STMT
        or node.kind == clang.cindex.CursorKind.CXX_CATCH_STMT
        or node.kind == clang.cindex.CursorKind.CXX_TRY_STMT
        or node.kind == clang.cindex.CursorKind.CXX_FOR_RANGE_STMT
    )


# Checks that a node is a type ref, the first child of a function that is a method will be a type ref.
# node the first child of a function outside a class
def is_type_ref(node):
    return node.kind == clang.cindex.CursorKind.TYPE_REF


# Is a function call
def is_call(node):
    return node.kind == clang.cindex.CursorKind.CALL_EXPR


# Check that a call is an on node
# Node must be a call
def is_on_call(node):
    return node.type.spelling == "NUClear::threading::ReactionHandle" and node.spelling == "then"


# Check that a call is an emit statement
# node must be a call
def is_emit_call(node):
    return node.spelling == "emit"


# Check that the node is a class decleration
def is_class(node):
    return node.kind == clang.cindex.CursorKind.CLASS_DECL


# The first child of a class decleration will have information about the parent if it is an child
def is_inherited(node, name):
    return node.kind == clang.cindex.CursorKind.CXX_BASE_SPECIFIER and node.type.spelling == name


# The datatype used for topological sort, represents a digraph
class AdjacencyList:
    def __init__(self):
        self.inner_list = {}
        self.size = 0

    def init_vertex(self, u):
        if not self.inner_list.get(u):
            self.inner_list[u] = []

    def add_edge(self, u, v):
        if not self.inner_list.get(u):
            self.inner_list[u] = []
        self.inner_list[u].append(v)

    def get(self, key):
        return self.inner_list.get(key)

    def __getitem__(self, key):
        return self.inner_list[key]

    def __len__(self):
        return self.size

    def __iter__(self):
        return iter(self.inner_list)

    def __repr__(self):
        out = ""
        for u in self:
            out += repr(u) + " -> " + repr([v for v in self.inner_list[u]]) + ","
        return out


# Topological sorts a given adjacency list
def topological_sort(adj):
    t_sorted = []
    visit = {}
    for u in adj:
        visit[u] = False

    for u in adj:
        if not visit[u]:
            _topological_sort_recurs(adj, u, t_sorted, visit)

    return t_sorted


# The recursive part of topological sort
def _topological_sort_recurs(adj, start, t_sorted, visit):
    visit[start] = True
    trav = adj[start]
    for trav_index in range(len(trav)):
        v = trav[trav_index]
        if not visit[v]:
            _topological_sort_recurs(adj, v, t_sorted, visit)

    t_sorted.append(start)
