import clang.cindex


class On:
    def __init__(self, node):
        self.node = node
        self.dsl, self.lmbda = self._findDSLAndLambda()

    def __repr__(self):
        return "on<{}>".format(self.dsl)

    def _findDSLAndLambda(self):
        DSL = ""
        lmbda = None
        children = self.node.get_children()
        try:
            for dslChild in next(
                next(next(next(children).get_children()).get_children()).get_children()
            ).get_children():
                if dslChild.kind == clang.cindex.CursorKind.TEMPLATE_REF:
                    DSL += "{}::".format(dslChild.spelling)
                elif dslChild.kind == clang.cindex.CursorKind.NAMESPACE_REF:
                    DSL += "{}::".format(dslChild.spelling)
                elif dslChild.kind == clang.cindex.CursorKind.TYPE_REF:
                    DSL += dslChild.type.spelling
        except StopIteration:
            pass
        return (DSL, lmbda)


class Emit:
    def __init__(self, node):
        self.node = node
        self.scope = self._findScope()

    def __repr__(self):
        return "emit<{}>".format(self.scope)

    def _findScope(self):
        scope = ""
        children = self.node.get_children()
        try:
            memberRefExpr = next(children)
            if memberRefExpr.kind != clang.cindex.CursorKind.MEMBER_REF_EXPR:
                raise AssertionError
            for part in memberRefExpr.get_children():
                if part.kind == clang.cindex.CursorKind.TEMPLATE_REF:
                    scope = part.spelling
            if scope == "":
                scope = "Local"
        except StopIteration:
            pass
        except AssertionError:
            print("First node in subtree of emit was not MEMBER_REF_EXPR, did not no was possible.")
        return scope


class Method:
    def __init__(self, node):
        self.node = node
        self.on = [On(on) for on in self._findOnNodes()]
        self.emit = [Emit(emit) for emit in self._findEmitNodes()]

    def __repr__(self):
        representation = self.node.spelling
        for on in self.on:
            representation += " " + str(on)
        for emit in self.emit:
            representation += " " + str(emit)
        return representation

    def _findOnNodes(self):
        ons = []
        for child in self.node.walk_preorder():
            if (
                child.kind == clang.cindex.CursorKind.CALL_EXPR
                and child.type.spelling == "NUClear::threading::ReactionHandle"
                and child.spelling == "then"
            ):
                ons.append(child)
        return ons

    # Find emit statement
    def _findEmitNodes(self):
        emits = []
        for child in self.node.walk_preorder():
            if child.kind == clang.cindex.CursorKind.CALL_EXPR and child.spelling == "emit":
                emits.append(child)
        return emits


class Reactor:
    def __init__(self, node, methods):
        self.node = node
        self.methods = [Method(method) for method in methods]

    def __repr__(self):
        reperesentation = self.node.spelling
        for method in self.methods:
            reperesentation += " {}".format(method)
        return reperesentation
