#!/usr/bin/python3

# clang --include-directory /home/nubots/build/shared --include-directory /home/nubots/NUbots/shared --include-directory /home/nubots/NUbots/nuclear/message/include -c -Xclang -ast-dump -fsyntax-only test.cpp
# pydoc -b tools/clang/cindex.py

import itertools
import multiprocessing
import os

import analyse
import b
from dockerise import run_on_docker

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
        if out[-1] != "[":
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
    if out[-1] != "[":
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
    if out[-1] != "[":
        out = out[:-1]
    out += "]}"
    return out


def parse(module, files, outdir):
    print("Working on module", module)

    with open(os.path.join(outdir, "_".join(module.split("/")) + ".json"), "w") as toWrite:
        toWrite.write(generateModuleJSON(analyse.createTree(files, module), module))


@run_on_docker
def register(command):
    command.help = "Generates documentation in JSON format for NUClear reactors and NUClear on and emit statements"

    command.add_argument("--outdir", default="doc/extract-documentation/", help="The output directory")

    command.add_argument(
        "--indir", default="module", help="The root of the directories that you want to scan. Default: module."
    )

    command.add_argument(
        "-m",
        "--multiprocess",
        dest="multiprocess",
        action="store_true",
        default=False,
        help="enable multiprocessing for faster generation, multiplies ram usage by thread count.",
    )


@run_on_docker
def run(outdir, indir, multiprocess, **kwargs):
    indir = os.path.abspath(indir)

    modules = {}

    srcDirs = []

    # Find all modules by walking the file tree
    for dirpath, dirnames, filenames in os.walk(indir):
        for dir in dirnames:
            if dir == "src":
                dirnames.remove("src")  # Don't look into the src directory
                srcDirs.append(os.path.join(dirpath, dir))

    # Find all the files in the source directories that aren't python files
    for srcDir in srcDirs:
        modules["/".join(srcDir.split("/")[0:-1])] = []
        for dirpath, dirnames, filenames in os.walk(srcDir):
            for filename in filenames:
                if not os.path.splitext(filename)[1] == ".py":
                    modules["/".join(srcDir.split("/")[0:-1])].append(os.path.join(dirpath, filename))

    # Loop through each module, looking for reactors then printing them
    if multiprocess:
        # Make sure we don't create more subprocesses than we need.
        process_count = min(multiprocessing.cpu_count(), len(modules))
        with multiprocessing.Pool(process_count) as pool:
            pool.starmap(
                parse, itertools.zip_longest(iter(modules.keys()), iter(modules.values()), iter([]), fillvalue=outdir)
            )
    else:
        for k, v in modules.items():
            parse(k, v, outdir)
