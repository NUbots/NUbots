import re


class Module:
    """
    A structure for holding information of a NUClearRoles module

    Attr:
        diagnostics: The diagnostics provided by clang, when it generated the AST
        reactors: Reactors present in this module
        functions: Functions present in this module that are interesting
        alias_stack: The
    """

    def __init__(self, diagnostics, reactors, functions):
        self.diagnostics = diagnostics
        self.reactors = reactors
        self.functions = functions
        self.alias_stack = []


class On:
    """
    A structure holding the information of a NUClear on statement

    Attr:
        node: The clang call statement node on the AST that this struct refers to
        dsl: The dsl that this on statement is on
        callback: The Function struct that holds the information of the on's callback
    """

    def __init__(self, node, dsl=None, callback=None):
        self.node = node
        self.dsl = dsl
        self.callback = callback


class Emit:
    """
    A structure holding the information of a NUClear emit statement

    Attr:
        node: The clang call statement node on the AST that this struct refers to
        scope: The scope that this emit has
        type: The type of the object that was emitted
    """

    # An existing unique pointer was passed in
    existing_unique_regex = re.compile(r"std::unique_ptr<(.*), std::default_delete<.*> >")

    # A new pointer was constructed in the emit
    make_unique_regex = re.compile(r"typename _MakeUniq<(.*)>::__single_object")

    # special case for `emit(graph("localisation ball pos", filter.get()[0], filter.get()[1]));`
    nusight_data_regex = re.compile(r"std::unique_ptr<(.*)>")

    def __init__(self, node, scope=None, type_=None):
        self.node = node
        self.scope = scope
        self.type = type_


class Function:
    """
    Information on functions within the module that have interesting information.
    Interesting functions have at least one on or emit or call another interesting function.

    Attr:
        node: The clang definition statement node on the AST that this struct refers to
        emits: A list of emits that this function emits
        ons: A list of ons that this function creates
        calls: A list of other functions that this one calls
    """

    def __init__(self, node):
        self.node = node
        self.emits = []
        self.ons = []
        self.calls = []


class Reactor:
    """
    A structure for holding information about reactors

    Attr:
        node: The clang definition statement node on the AST that this struct refers to
        methods: A list of interesting functions that this reactor has
    """

    def __init__(self, node, methods):
        self.node = node
        self.methods = methods

    def get_name(self):
        """ Returns the name defined for this reactor"""
        return self.node.type.spelling


class Alias:
    """
    Information on a type alias
    This usually comes from a using statement

    Attr:
        node: The clang definition statement node on the AST that this struct refers to
        original: The original type name that was aliased
        aliased: The new name used in place of the original
    """

    def __init__(self, node, original, aliased):
        self.node = node
        self.original = original
        self.aliased = aliased
