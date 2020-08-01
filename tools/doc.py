#!/usr/bin/python3

# clang --include-directory /home/nubots/build/shared --include-directory /home/nubots/NUbots/shared --include-directory /home/nubots/NUbots/nuclear/message/include -c -Xclang -ast-dump -fsyntax-only test.cpp
# pydoc -b tools/clang/cindex.py

import sys
import os

import b
from dockerise import run_on_docker
import analyse


def generateReactorMarkdown(reactor):
    out = "# {}".format(reactor.getType())
    out += "\n{}".format(reactor.getBrief())
    for _, method in reactor.getMethodsNoDuplicate().items():
        out += "\n## {}".format(method.getName())
        out += "\n{}".format(method.getBrief())
        for on in method.getOns():
            out += "\n* {}".format(str(on).replace("<", "\<"))
            out += " {}".format(on.getBrief())
        for emit in method.getEmits():
            out += "\n* {}".format(str(emit).replace("<", "\<"))
            out += " {}".format(emit.getBrief())
    out += "\n"
    return out


@run_on_docker
def register(command):
    command.help = "documentation generation?"

    command.add_argument("path", help="The file to do")

    command.add_argument("outpath", help="The file to write to")


@run_on_docker
def run(path, outpath, **kwargs):
    index = analyse.createIndex()
    tu = analyse.translate(index, path)

    # Check that the parsing of the tu succeeded
    failed = False
    for diagnostic in tu.getDiagnostics():
        print(diagnostic)
        failed = True
    if failed:
        return

    toWrite = open(outpath, "w")

    reactors = tu.getReactors()
    for reactor in reactors:
        toWrite.write(generateReactorMarkdown(reactor))

    toWrite.close()

    # analyse.printReactorAst(reactors[-1])
