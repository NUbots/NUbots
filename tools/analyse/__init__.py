import os.path
import re
from sys import stderr
from itertools import chain

import clang.cindex

from .struct import Alias, Emit, Function, On, Reactor, Module
from .topological_sort import AdjacencyList, topological_sort

# The path to the folder that contains libclang
library_path = "/usr/local/lib"  # llvm-config --libdir
clang.cindex.Config.set_library_path(library_path)

# The arguments to pass to clang
parse_args = [
    "-I../build/shared",
    "-Ishared",
    "-Inuclear/message/include",
    "-I/usr/local/lib/clang/11.0.1/include",  # clang include path, clang -E -v -
    "-I/usr/local/include/eigen3",
    "-I/usr/local/include/aravis-0.8/",
    "-I/usr/local/include/glib-2.0/",
    "--std=c++17",
]

# A list of folders that could have interesting things
# These folders should have c++ files that have reactors or call emit and/or on in the powerplant
whitelist = [
    "/usr/local/include/nuclear_bits/",
]


def repr_node(node, tabs=0):
    """
    Formats one node in the clang AST.
    nodetype type#name file:line:column

    Args:
        node (clang.cindex.cursor): A node in the clang AST
        tabs (int, optional): The amount this should be indented. Defaults to 0.

    Returns:
        str: A string with information on a node
    """
    return "  " * tabs + "{} {}#{} {}:{}:{}".format(
        node.kind.name, node.type.spelling, node.spelling, node.location.file, node.location.line, node.location.column
    )


def repr_tree(node, tabs=0):
    """
    Generates a string so you can read the AST from a given node

    Args:
        node (clang.cindex.cursor): A node in the clang AST
        tab (int, optional): The amount this should be indented. Defaults to 0.

    Returns:
        str: A string with the ast in a kinda human readable form
    """
    out = ""
    out += repr_node(node, tabs) + "\n"
    for child in node.get_children():
        out += repr_tree(child, tabs + 1)
    return out


def parse_module(files, folder):
    """Parses all files in a folder for interesting information

    Args:
        files (str): A list of files to parse
        folder (str): The folder the files reside in

    Returns:
        struct.Module: All the interesting information that was in the files
    """
    index = clang.cindex.Index.create()
    translation_units = {}
    diagnostics = []

    # Construct AST for each file
    total = len(files)
    for i, f in enumerate(files):
        f = os.path.abspath(f)
        print("  [{}/{}] generating AST for file".format(i + 1, total), f)
        translation_units[f] = index.parse(f, parse_args + ["-I" + folder + "/src"])
        diagnostics += list(translation_units[f].diagnostics)

    module = Module(diagnostics, [], {})

    # Make sure that there are no errors in the parsing
    for diagnostic in diagnostics:
        if diagnostic.severity >= clang.cindex.Diagnostic.Error:
            print(diagnostic, file=stderr)
            return module

    # Create an adjacency list of the include digraph for the topological sort
    adj_list = AdjacencyList()

    # Add edges to the adjacency list by looking at the includes
    for tu in translation_units.values():
        current_file = os.path.abspath(tu.spelling)
        adj_list.init_vertex(current_file)
        for file_inclusion in tu.get_includes():
            include_path = os.path.abspath(file_inclusion.include.name)
            if os.path.commonpath([include_path, folder]) == folder:
                adj_list.add_edge(current_file, include_path)

    # Topological sort the adjacency list so we parse files before the ones that include them
    order = topological_sort(adj_list)

    # Go through the files, pulling out relevant information
    total = len(order)
    for i, file_name in enumerate(order):
        print("  [{}/{}] working on file".format(i + 1, total), file_name)
        # For the initial loop through make sure that the file the node comes from is whitelisted or in the given folder
        for child in translation_units[file_name].cursor.get_children():
            child_path = os.path.abspath(child.location.file.name)

            if os.path.commonpath([child_path, file_name]) == file_name:
                module.alias_stack.append([])
                _tree_parse(child, module)
                module.alias_stack.pop()

    return module


def _tree_parse(node, module):
    """
    Look at the node, if it is a reactor, function or alias parse it, else recursively loop over it's children

    Args:
        node (clang.cindex.Cursor): The node who we will parse
        module (struct.Module): The module struct for this module
    """
    # Work out where the start of the scope is so we know where our using statements are meant to be
    scope_start = False
    if is_scope_start(node):
        module.alias_stack.append([])
        scope_start = True

    if is_function(node):
        function = make_function(node, module)
        if function:
            module.functions[function.node.displayname] = function
    elif is_class(node):
        try:
            if is_inherited(next(node.get_children()), "NUClear::Reactor"):
                module.reactors.append(make_reactor(node, module))
            else:
                for child in node.get_children():
                    _tree_parse(child, module)
        except StopIteration as e:
            for child in node.get_children():
                _tree_parse(child, module)  # Class was a forward declare with no inheritance
    elif is_using(node):
        module.alias_stack[-1].append(parse_alias(node))
    else:
        for child in node.get_children():
            _tree_parse(child, module)

    # If the node was the start of a scope, only it's children will be in that scope
    if scope_start:
        module.alias_stack.pop()


def make_function(node, module):
    """
    Parse a node that defines a function

    Args:
        node (clang.cindex.Cursor): A node that refers to a definition of a function
        module (struct.Module): The module struct for this module

    Returns:
        struct.Function or None: A struct with the information of the function or None if the function was not
                                 interesting i.e. did not have an on, emit or did not call another interesting function.
    """
    function = Function(node)

    _function_parse(node, function, module)

    # Find if the function was a method
    # Assuming you can't declare a method before declaring a class in c++, this code works
    try:
        child = next(node.get_children())
        # first child of a method will be a type ref.
        if is_type_ref(child):
            for reactor in module.reactors:
                if child.type.spelling == reactor.node.type.spelling:
                    reactor.methods.append(function)
    except StopIteration as e:
        pass  # Function was a forward declared and not a method

    # Check that the function is interesting i.e. has an emit, on or calls another interesting function
    if (not function.emits == []) or (not function.ons == []) or (not function.calls == []):
        return function


def _function_parse(node, function, module):
    """
    Recursively loop through the AST finding information about a function

    Args:
        node (clang.cindex.Cursor): A node that is a child of the function
        function (struct.Function): The struct that we will save the information we find
        module (struct.Module): The module struct for this module
    """
    for child in node.get_children():
        if is_call(child):
            if is_on_call(child):
                function.ons.append(make_on(child, module))
            elif is_emit_call(child):
                function.emits.append(make_emit(child, module))
            else:
                definition = child.get_definition()
                if definition:
                    called_function = module.functions.get(definition.displayname)
                    # If the function is calling another function, if the other function is in the whitelist
                    # or if the other function is already parsed, give it a look
                    if not called_function:
                        # A new function that we have not parsed yet
                        for file_name in whitelist:
                            if (
                                os.path.commonpath([os.path.abspath(definition.location.file.name), file_name])
                                == file_name
                            ):
                                called_function = make_function(definition, module)
                                if called_function:
                                    module.functions[called_function.node.displayname] = called_function
                                    function.calls.append(called_function)
                                    function.calls.extend(called_function.calls)
                                break
                    else:
                        # A function we have already parsed
                        function.calls.append(called_function)
                        function.calls.extend(called_function.calls)

        else:
            _function_parse(child, function, module)


def make_on(node, module):
    """
    Parse a node that defines an on statement

    Args:
        node (clang.cindex.Cursor): The cursor that calls the on function
        module (struct.Module): The module struct for this module

    Returns:
        struct.On: The struct holding the information about this on statement
    """
    on = On(node)
    children = node.get_children()

    # Find the dsl type by traversing the tree. The tree for an on statement has a predictable structure.
    try:
        dsl = ""
        # Journey to the part of the on statement that defines the type of the on statement
        for dsl_child in next(next(next(next(children).get_children()).get_children()).get_children()).get_children():
            # Build up the full typename by looping through the series of types references
            if dsl_child.kind == clang.cindex.CursorKind.TEMPLATE_REF:
                dsl += apply_aliases(f"{dsl_child.spelling}::", chain.from_iterable(module.alias_stack))
            elif dsl_child.kind == clang.cindex.CursorKind.NAMESPACE_REF:
                dsl += apply_aliases(f"{dsl_child.spelling}::", chain.from_iterable(module.alias_stack))
            elif dsl_child.kind == clang.cindex.CursorKind.TYPE_REF:
                dsl += apply_aliases(dsl_child.type.spelling, chain.from_iterable(module.alias_stack))
        on.dsl = dsl
    except StopIteration as e:
        print("On DSL find StopIter:", e)

    # Find the callback for the function
    try:
        found = False
        for callback_child in children:
            if found:
                break
            if callback_child.kind == clang.cindex.CursorKind.UNEXPOSED_EXPR:  # lambda
                callback = next(callback_child.get_children())
                function = make_function(callback, module)
                if function:
                    on.callback = function
                    found = True
            elif callback_child.kind == clang.cindex.CursorKind.DECL_REF_EXPR:  # reference to predefined function
                for function in module.functions.values():
                    if function.node == callback_child.referenced:
                        on.callback = function
                        found = True
                        break
    except StopIteration as e:
        print("On callback find StopIter:", e)

    return on


def make_emit(node, module):
    """
    Parse a node that defines an emit statement

    Args:
        node (clang.cindex.Cursor): The cursor that calls the emit function
        module (struct.Module): The module struct for this module

    Returns:
        struct.Emit: The struct holding the information about this emit statement
    """
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

    emit.type = apply_aliases(emit.type, chain.from_iterable(module.alias_stack))

    return emit


def make_reactor(node, module):
    """
    Parse a node that defines an reactor definition

    Args:
        node (clang.cindex.Cursor): The node that defines the reactor
        module (struct.Module): The module struct for this module

    Returns:
        struct.Reactor: The struct holding the information about this reactor
    """
    reactor = Reactor(node, [])

    # Find if this reactor is forward declared
    for candidate_reactor in module.reactors:
        if candidate_reactor.node.type.spelling == node.type.spelling:
            reactor = candidate_reactor
            reactor.node = node

    # Find the methods in the reactor
    for child in node.get_children():
        if is_call(child):
            function = make_function(child, module)
            if function:
                reactor.methods.append(function)
                module.functions[function.node.displayname] = function

    return reactor


def parse_alias(node):
    """
    Parse using statements to get the type aliases

    Args:
        node (clang.cindex.Cursor): The node that defines this alias

    Returns:
        struct.Alias: The struct that holds the information of this alias
    """
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
            print("Did not find TYPE_REF under TYPE_ALIAS_DECL")
        for _ in range(template_level):
            original += ">"
    elif node.kind == clang.cindex.CursorKind.NAMESPACE_ALIAS:
        print(
            "A namespace alias was used, this is currently not implemented as I cannot find an example in the codebase"
        )

    return Alias(node, original, aliased)


def apply_aliases(dsl, aliases):
    """
    Apply the aliases to the type

    Args:
        dsl (str): The type to be resolved into its proper type
        aliases (iter(struct.Alias)): An iterator of aliases to apply

    Returns:
        str: The type after aliases have been applied
    """
    # If the type was aliased, instead store the original type
    for alias in aliases:
        if dsl == alias.aliased:
            dsl = alias.original
            break

    return dsl


def is_function(node):
    """
    Checks if the node is a function definition

    Args:
        node (clang.cindex.Cursor): The node to check

    Returns:
        bool: If the node is a function definition
    """
    return (
        node.kind == clang.cindex.CursorKind.CXX_METHOD
        or node.kind == clang.cindex.CursorKind.CONSTRUCTOR
        or node.kind == clang.cindex.CursorKind.DESTRUCTOR
        or node.kind == clang.cindex.CursorKind.FUNCTION_DECL
        or node.kind == clang.cindex.CursorKind.CONVERSION_FUNCTION
    )


def is_using(node):
    """
    Checks if the node is a using statement

    Args:
        node (clang.cindex.Cursor): The node to check

    Returns:
        bool: If the node is a using statement
    """
    return (
        node.kind == clang.cindex.CursorKind.USING_DECLARATION
        or node.kind == clang.cindex.CursorKind.TYPE_ALIAS_DECL
        or node.kind == clang.cindex.CursorKind.NAMESPACE_ALIAS
    )


def is_scope_start(node):
    """
    Checks if the node starts a new scope

    Args:
        node (clang.cindex.Cursor): The node to check

    Returns:
        bool: If the node starts a new scope
    """
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


def is_type_ref(node):
    """
    Checks that a node is a reference to a type

    Args:
        node (clang.cindex.Cursor): The node to check

    Returns:
        bool: If the node is a type reference
    """
    return node.kind == clang.cindex.CursorKind.TYPE_REF


def is_call(node):
    """
    Checks that a node is a function call

    Args:
        node (clang.cindex.Cursor): The node to check

    Returns:
        bool: If the node is a function call
    """
    return node.kind == clang.cindex.CursorKind.CALL_EXPR


def is_on_call(node):
    """
    Checks that a node is an on call

    Args:
        node (clang.cindex.Cursor): The function call node to check

    Returns:
        bool: If the function call node is a on call
    """
    return node.type.spelling == "NUClear::threading::ReactionHandle" and node.spelling == "then"


def is_emit_call(node):
    """
    Checks that a node is an emit call

    Args:
        node (clang.cindex.Cursor): The function call node to check

    Returns:
        bool: If the function call node is a emit call
    """
    return node.spelling == "emit"


def is_class(node):
    """
    Checks that a node is a class definition

    Args:
        node (clang.cindex.Cursor): The node to check

    Returns:
        bool: If the node is a class definition
    """
    return node.kind == clang.cindex.CursorKind.CLASS_DECL


def is_inherited(node, name):
    """
    Checks that a node is a base specifier for the given type. A base specifier, specifies what the class inherits

    Args:
        node (clang.cindex.Cursor): The node to check, that is the first child of a class definition
        name (str): The type to check inheritance for

    Returns:
        bool: If the node is a base specifier for the type given by name
    """
    return node.kind == clang.cindex.CursorKind.CXX_BASE_SPECIFIER and node.type.spelling == name
