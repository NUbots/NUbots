#!/usr/bin/python

import pyparsing as pp

# Enable packrat for fastness!
pp.ParserElement.enablePackrat()


class SymbolParser:
    def locator(self, s, l, t):
        return l if self.locate else None

    def __init__(self):

        # Match an operator declaration
        operatorType = pp.Group(
            pp.Literal("operator")
            + (
                pp.Literal("~")
                | pp.Literal("||")
                | pp.Literal("|=")
                | pp.Literal("|")
                | pp.Literal(">>=")
                | pp.Literal(">>")
                | pp.Literal(">=")
                | pp.Literal(">")
                | pp.Literal("==")
                | pp.Literal("=")
                | pp.Literal("<=")
                | pp.Literal("<<=")
                | pp.Literal("<<")
                | pp.Literal("<")
                | pp.Literal("+=")
                | pp.Literal("++")
                | pp.Literal("+")
                | pp.Literal("^=")
                | pp.Literal("^")
                | pp.Literal("%=")
                | pp.Literal("%")
                | pp.Literal("&=")
                | pp.Literal("&&")
                | pp.Literal("&")
                | pp.Literal("/=")
                | pp.Literal("/")
                | pp.Literal("*=")
                | pp.Literal("*")
                | pp.Literal("[]")
                | pp.Literal("()")
                | pp.Literal("!=")
                | pp.Literal("!")
                | pp.Literal(",")
                | pp.Literal("->*")
                | pp.Literal("->")
                | pp.Literal("-=")
                | pp.Literal("--")
                | pp.Literal("-")
            )
        )

        # Fundamental types (types built into c++)
        fundamentalType = (
            pp.Literal("bool")
            | pp.Literal("unsigned char")
            | pp.Literal("signed char")
            | pp.Literal("char")
            | pp.Literal("short int")
            | pp.Literal("short")
            | pp.Literal("int")
            | pp.Literal("signed short int")
            | pp.Literal("signed short")
            | pp.Literal("signed int")
            | pp.Literal("signed")
            | pp.Literal("unsigned short int")
            | pp.Literal("unsigned short")
            | pp.Literal("unsigned int")
            | pp.Literal("unsigned")
            | pp.Literal("long long int")
            | pp.Literal("long long")
            | pp.Literal("long int")
            | pp.Literal("long")
            | pp.Literal("signed long long int")
            | pp.Literal("signed long long")
            | pp.Literal("signed long int")
            | pp.Literal("signed long")
            | pp.Literal("unsigned long long int")
            | pp.Literal("unsigned long long")
            | pp.Literal("unsigned long int")
            | pp.Literal("unsigned long")
        )

        # Type modifier, e.g. const volatile && * etc
        typeModifiers = pp.Group(
            pp.ZeroOrMore(pp.Literal("const") | pp.Literal("volatile") | pp.Literal("&") | pp.Literal("*"))
        )

        # Match a cType alphanum string, also matches int constants since we don't restrict the forst char
        cType = pp.Word(pp.alphanums + "_")

        # Array type e.g. [8]
        arrayType = pp.Regex(r"\[\d+\]")

        # The function/array modifier instance the '(&)' or '(*)' that is there
        funcOrArrayModifier = pp.Regex(r"\([&*]+\)")

        # trailing information of form [clone .lto_priv.1145]
        trailingInfo = pp.ZeroOrMore(pp.Suppress(pp.Literal("[") + pp.Regex(r"[^\]]+") + pp.Literal("]")))

        # A locator system that can be turned on to put the char index in the parsed output
        locator = pp.Empty().setParseAction(self.locator)

        # TYPES THAT DEPEND ON RECURSION!!!

        # Namespaced type (potentially containing templates) e.g. `a::b::c<x::y>`
        nsType = pp.Forward()

        # An enum type e.g. `(a::b::c)1`
        enumType = pp.Suppress("(") + nsType + pp.Suppress(")") + pp.Word(pp.nums)

        # Match a template (list of types, enums and empties)
        templateType = (
            pp.Suppress("<")
            + pp.Optional(pp.Group(pp.delimitedList(pp.Group(enumType | nsType | pp.Empty()))))
            + pp.Suppress(">")
        )

        # A function call e.g. (list, of, args)
        funcCall = pp.Suppress("(") + pp.Optional(pp.Group(pp.delimitedList(pp.Group(nsType)))) + pp.Suppress(")")

        # A lambda type e.g. {lambda(a,b,c)#1} or {parm#2}
        lambdaType = (
            pp.Suppress("{")
            + pp.Group(cType + pp.Optional(funcCall) + pp.Suppress("#") + pp.Word(pp.nums))
            + pp.Suppress("}")
        )

        # grouping using parens
        parensGroup = pp.Suppress("(") + pp.Group(nsType) + pp.Suppress(")")

        # Fill ns type (which is made up of several types separated by ::)
        nsType << locator + pp.delimitedList(
            pp.Suppress(typeModifiers)
            + (lambdaType | fundamentalType | operatorType | cType | parensGroup)
            + pp.Suppress(typeModifiers)
            + pp.Optional(templateType)
            + pp.Suppress(typeModifiers)
            + pp.Optional(funcOrArrayModifier)
            + pp.Suppress(typeModifiers)
            + pp.Optional(arrayType)
            + pp.Suppress(typeModifiers)
            + pp.Optional(funcCall)
            + pp.Suppress(typeModifiers),
            pp.Literal("::") | pp.Literal("."),
        ) + locator

        self.parser = pp.OneOrMore(pp.Group(nsType)) + trailingInfo + pp.StringEnd()
        self.locate = False

    def parse_symbol(self, symbol, locateTags=False):
        self.locate = locateTags
        return self.parser.parseString(symbol).asList()
