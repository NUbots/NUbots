#!/usr/bin/python3

# clang --include-directory /home/nubots/build/shared --include-directory /home/nubots/NUbots/shared --include-directory /home/nubots/NUbots/nuclear/message/include -c -Xclang -ast-dump -fsyntax-only test.cpp
# pydoc -b tools/clang/cindex.py

import sys
import os

import b
from dockerise import run_on_docker
import analyse

# from clang.cindex import SourceLocation


def generateLocationJSON(location):
    out = "{"
    out += '"file":"{}",'.format(location.file)
    out += '"line":"{}",'.format(location.line)
    out += '"column":"{}",'.format(location.column)
    out += '"offset":"{}"'.format(location.offset)
    out += "}"
    return out


def generateEmitJSON(emit):
    out = "{"
    out += '"scope":"{}",'.format(emit.scope)
    out += '"type":"{}",'.format(emit.type)
    out += '"location":{}'.format(generateLocationJSON(emit.node.location))
    out += "}"
    return out


def generateOnJSON(on):
    out = "{"
    out += '"dsl":"{}",'.format(on.dsl)
    out += '"emit":['
    if on.callback and on.callback.calls:
        for call in on.callback.calls:
            for emit in call.emits:
                out += generateEmitJSON(emit)
                out += ","
        out = out[:-1]
    out += "],"
    out += '"location":{}'.format(generateLocationJSON(on.node.location))
    out += "}"
    return out


def generateReactorJSON(reactor):
    out = "{"
    out += '"name":"{}",'.format(reactor.getName())
    out += '"on":['
    for method in reactor.methods:
        for on in method.ons:
            out += generateOnJSON(on) + ","
    out = out[:-1]
    out += "],"
    out += '"location":{}'.format(generateLocationJSON(reactor.node.location))
    out += "}"
    return out


def generateModuleJSON(tree, name):
    out = "{"
    out += '"name":"{}",'.format(name)
    out += '"reactors":['
    for reactor in tree.reactors:
        out += generateReactorJSON(reactor)
        out += ","
    out = out[:-1]
    out += "]}"
    return out


@run_on_docker
def register(command):
    command.help = "documentation generation?"

    command.add_argument("--outdir", default="doc/NUdoc/", help="The output directory")

    command.add_argument(
        "--indir", default="module", help="The root of the directories that you want to scan. Default: module."
    )


@run_on_docker
def run(outdir, indir, **kwargs):
    indir = os.path.abspath(indir)

    modules = {}

    # Find all modules and each file in them by walking the file tree
    for dirpath, dirnames, filenames in os.walk(indir):
        if dirpath.split("/")[-1] == "src":
            modules["/".join(dirpath.split("/")[0:-1])] = []
            for filename in filenames:
                modules["/".join(dirpath.split("/")[0:-1])].append(os.path.join(dirpath, filename))

    # Loop through each module, looking for reactors then printing them
    for module, files in modules.items():
        print("Working on module", module)

        toWrite = open(os.path.join(outdir, "_".join(module.split("/")) + ".json"), "w")

        toWrite.write(generateModuleJSON(analyse.createTree(files, module), module))

        toWrite.close()
