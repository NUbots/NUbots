class Reactor:
    def __init__(self, node, methods):
        self.node = node
        self.methods = methods

    def __repr__(self):
        reperesentation = self.node.spelling
        for method in self.methods:
            reperesentation += " {}".format(method.spelling)
        return reperesentation

    # def _findOnNodes(self, node):
    #    for child in node.walk_preorder():
    #        if child.kind == clang.cindex.CursorKind.CALL_EXPR and child.spelling == "emit":
    #            yield child

    # Find emit statement
    # def _findEmitNodes(self, node):
    #    for child in node.walk_preorder():
    #        if child.kind == clang.cindex.CursorKind.CALL_EXPR and child.spelling == "emit":
    #            yield child
