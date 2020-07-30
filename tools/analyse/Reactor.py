from .Class import Class


class Reactor(Class):
    def __init__(self):
        pass

    # def _findOnNodes(self, node):
    #    for child in node.walk_preorder():
    #        if child.kind == clang.cindex.CursorKind.CALL_EXPR and child.spelling == "emit":
    #            yield child

    # Find emit statement
    # def _findEmitNodes(self, node):
    #    for child in node.walk_preorder():
    #        if child.kind == clang.cindex.CursorKind.CALL_EXPR and child.spelling == "emit":
    #            yield child
