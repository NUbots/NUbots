#!/usr/bin/python3

# clang --include-directory /home/nubots/build/shared --include-directory /home/nubots/NUbots/shared --include-directory /home/nubots/NUbots/nuclear/message/include -c -Xclang -ast-dump -fsyntax-only test.cpp
# pydoc -b tools/clang/cindex.py

import sys
import os

import b
from dockerise import run_on_docker
import analyse
from clang.cindex import Diagnostic

# Returns a formatted string of  the given reactor
def generateReactorMarkdown(reactor):
    out = "## {}".format(reactor.getType())
    out += "\n{}".format(reactor.getBrief())
    for _, method in reactor.getMethodsNoDuplicate().items():
        out += "\n### {}".format(method.getName())
        out += "\n{}".format(method.getBrief())
        for on in method.getOns():
            out += "\n* {}".format(str(on).replace("<", "\<"))
            out += " {}".format(on.getBrief())
        for emit in method.getEmits():
            out += "\n* {}".format(str(emit).replace("<", "\<"))
            out += " {}".format(emit.getBrief())
    out += "\n"
    return out


# Returns a formatted string of the given module and all reactors in it
def generateModuleMarkdown(module, reactors):
    out = "# " + "::".join(module.split("/"))
    for reactor in reactors:
        out += "\n" + generateReactorMarkdown(reactor)
    out += "\n"
    return out


@run_on_docker
def register(command):
    command.help = "documentation generation?"

    command.add_argument("outdir", help="The output directory")

    command.add_argument(
        "--indir", default="module", help="The root of the directories that you want to scan. Default: module."
    )


@run_on_docker
def run(outdir, indir, **kwargs):
    index = analyse.createIndex()

    # toWrite = open("out.txt", "w")
    # toWrite.write(analyse.printTree(analyse.translate(index, indir).translationUnit.cursor))
    # toWrite.close()
    # return

    modules = {}

    # Find all modules and each file in them by walking the file tree
    for dirpath, dirnames, filenames in os.walk(indir):
        if dirpath.split("/")[-1] == "src":
            modules["/".join(dirpath.split("/")[0:-1])] = filenames

    # Loop through each module, looking for reactors then printing them
    for module, files in modules.items():
        print("Working on module", module)
        reactors = []
        for f in files:
            if os.path.splitext(f)[1] == ".cpp":
                print("    Working on file", f)
                translationUnit = analyse.translate(index, os.path.join(module, "src", f))

                # Check that the parsing of the tu succeeded
                failed = False
                for diagnostic in translationUnit.getDiagnostics():
                    # If the diagnostic is an error or fatal, print and don't look at the errornous translation unit
                    if diagnostic.severity >= Diagnostic.Error:
                        print(diagnostic, ", ", module, "/src/", f, " will not be looked at", sep="")
                        failed = True
                if failed:
                    continue

                reactors += translationUnit.getReactors()

        toWrite = open(os.path.join(outdir, "_".join(module.split("/")) + ".md",), "w")
        toWrite.write(generateModuleMarkdown(module, reactors))
        toWrite.close()
