import clang.cindex


class On:
    def __init__(self, node):
        self.node = node
        self.dsl = self._findDSL()
        self.lmbda = self._findLambda()

    def __repr__(self):
        return "on<{}>(){{{}}}".format(self.dsl, self.lmbda)

    def _findDSL(self):
        DSL = ""
        try:
            for dslChild in next(
                next(next(next(self.node.get_children()).get_children()).get_children()).get_children()
            ).get_children():
                if dslChild.kind == clang.cindex.CursorKind.TEMPLATE_REF:
                    DSL += "{}::".format(dslChild.spelling)
                elif dslChild.kind == clang.cindex.CursorKind.NAMESPACE_REF:
                    DSL += "{}::".format(dslChild.spelling)
                elif dslChild.kind == clang.cindex.CursorKind.TYPE_REF:
                    DSL += dslChild.type.spelling
        except StopIteration:
            pass
        return DSL

    def _findLambda(self):
        for child in self.node.get_children():
            try:
                if child.kind == clang.cindex.CursorKind.UNEXPOSED_EXPR:
                    lmbda = next(child.get_children())
                    if lmbda.kind != clang.cindex.CursorKind.LAMBDA_EXPR:
                        raise AssertionError("Was not a lambda")
                    else:
                        return Method(lmbda)
            except StopIteration:
                pass
            except AssertionError as e:
                print(e)
        return None


class Emit:
    def __init__(self, node):
        self.node = node
        self.scope = self._findScope()
        self.type = self._findType()

    def __repr__(self):
        return "emit<{}>({})".format(self.scope, self.type)

    def __eq__(self, value):
        return self.node == value.node

    def _findScope(self):
        try:
            memberRefExpr = next(self.node.get_children())
            if memberRefExpr.kind != clang.cindex.CursorKind.MEMBER_REF_EXPR:
                raise AssertionError("First node in subtree of emit was not MEMBER_REF_EXPR, did not no was possible.")
            for part in memberRefExpr.get_children():
                if part.kind == clang.cindex.CursorKind.TEMPLATE_REF:
                    return part.spelling
        except StopIteration:
            pass
        except AssertionError as e:
            print(e)
        return "Local"

    def _findType(self):
        try:
            children = self.node.get_children()
            next(children)
            expr = next(children)
            if expr.kind == clang.cindex.CursorKind.UNEXPOSED_EXPR:
                for child in next(
                    next(next(next(expr.get_children()).get_children()).get_children()).get_children()
                ).get_children():
                    if child.kind == clang.cindex.CursorKind.TYPE_REF:
                        return child.type.spelling
        except StopIteration:
            pass
        return ""


class Method:
    def __init__(self, node):
        self.node = node
        self.calls = [node]
        self.emit = []
        self.on = self._findOnNodes()
        self._addEmitNodes()

    def __repr__(self):
        representation = self.node.spelling + "("
        for on in self.on:
            representation += " " + str(on)
        for emit in self.emit:
            representation += " " + str(emit)
        representation += ")"
        return representation

    def _findOnNodes(self):
        ons = []
        for child in self.node.walk_preorder():
            if (
                child.kind == clang.cindex.CursorKind.CALL_EXPR
                and child.type.spelling == "NUClear::threading::ReactionHandle"
                and child.spelling == "then"
            ):
                on = On(child)
                for emit in on.lmbda.emit:
                    if emit not in self.emit:
                        self.emit.append(emit)
                ons.append(on)
        return ons

    # Find and add emit statement
    def _addEmitNodes(self):
        for child in self.node.walk_preorder():
            if child.kind == clang.cindex.CursorKind.CALL_EXPR and child.spelling == "emit":
                emit = Emit(child)
                if emit not in self.emit:
                    self.emit.append(emit)
            elif (
                child.kind == clang.cindex.CursorKind.CALL_EXPR
                and child.get_definition()
                and child.get_definition() not in self.calls
            ):
                self.calls.append(child.get_definition())
                self._addEmitFunction(child.get_definition())

    def _addEmitFunction(self, node):
        for child in node.walk_preorder():
            if child.kind == clang.cindex.CursorKind.CALL_EXPR and child.spelling == "emit":
                emit = Emit(child)
                if emit not in self.emit:
                    self.emit.append(emit)
            elif (
                child.kind == clang.cindex.CursorKind.CALL_EXPR
                and child.get_definition()
                and child.get_definition() not in self.calls
            ):
                self.calls.append(child.get_definition())
                self._addEmitFunction(child.get_definition())


class Reactor:
    def __init__(self, node, methods):
        self.node = node
        self.methods = [Method(method) for method in methods]

    def __repr__(self):
        reperesentation = self.node.spelling
        for method in self.methods:
            reperesentation += " {}".format(method)
        return reperesentation
