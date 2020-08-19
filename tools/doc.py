#!/usr/bin/python3

# clang --include-directory /home/nubots/build/shared --include-directory /home/nubots/NUbots/shared --include-directory /home/nubots/NUbots/nuclear/message/include -c -Xclang -ast-dump -fsyntax-only test.cpp
# pydoc -b tools/clang/cindex.py

import sys
import os

import b
from dockerise import run_on_docker
import analyse
from clang.cindex import Diagnostic


def generateOnJSON(on):
    calls = on.callback
    emits = on.callback.emits

    for call in on.callback.calls:
        calls.append(call)
        emits.extend(call.emits)

    out = "{"
    out += "dsl:{}".format(on.dsl)
    out += "}"
    return out


def generateReactorJSON(reactor):
    out = "{"
    out += "name:{},".format(reactor.getName())
    out += "on:["
    for method in reactor.methods:
        for on in method.ons:
            out += generateOnJSON(on) + ","
    out = out[:-1]
    out += "]"
    out += "}"
    return out


def generateModuleJSON(module, reactors):
    out = "{"
    out += "name:{}".format(module)
    out += "["
    for reactor in reactors:
        out += generateReactorJSON(reactor) + ","
    out = out[:-1]
    out += "]"
    out += "}"
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

                tree = createTree(index, f)

                reactors.extend(translationUnit.reactors())

        toWrite = open(os.path.join(outdir, "_".join(module.split("/")) + ".md",), "w")
        toWrite.write(generateModuleJSON(module, reactors))
        toWrite.close()
