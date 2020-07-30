import clang.cindex


class Method:
    def __init__(self, node):
        self.node = node
        self.on = self._findOnNodes(node)
        self.emit = self._findEmitNodes(node)

    def __repr__(self):
        representation = self.node.spelling
        for on in self.on:
            representation += " on:{}:{}:{}".format(on.location.file, on.location.line, on.location.column)
        for emit in self.emit:
            representation += " emit:{}:{}:{}".format(emit.location.file, emit.location.line, emit.location.column)
        return representation

    def _findOnNodes(self, node):
        ons = []
        for child in node.walk_preorder():
            if (
                child.kind == clang.cindex.CursorKind.CALL_EXPR
                and child.type.spelling == "NUClear::threading::ReactionHandle"
                and child.spelling == "then"
            ):
                ons.append(child)
        return ons

    # Find emit statement
    def _findEmitNodes(self, node):
        emits = []
        for child in node.walk_preorder():
            if child.kind == clang.cindex.CursorKind.CALL_EXPR and child.spelling == "emit":
                emits.append(child)
        return emits


class on:
    def __init__(self, node):
        self.node = node


class Reactor:
    def __init__(self, node, methods):
        self.node = node
        self.methods = [Method(method) for method in methods]

    def __repr__(self):
        reperesentation = self.node.spelling
        for method in self.methods:
            reperesentation += " {}".format(method)
        return reperesentation
