#!/usr/bin/python3

from io import SEEK_END
from multiprocessing import Pool, cpu_count, Lock
from itertools import repeat
import os

import analyse
import b
from dockerise import run_on_docker


def generate_location_JSON(location):
    out = "{"
    out += '"file":"{}",'.format(location.file)
    out += '"line":"{}",'.format(location.line)
    out += '"column":"{}",'.format(location.column)
    out += '"offset":"{}"'.format(location.offset)
    out += "}"
    return out


def generate_emit_JSON(emit):
    out = "{"
    out += '"scope":"{}",'.format(emit.scope)
    out += '"type":"{}",'.format(emit.type)
    out += '"location":{}'.format(generate_location_JSON(emit.node.location))
    out += "}"
    return out


def generate_on_JSON(on):
    out = "{"
    out += '"dsl":"{}",'.format(on.dsl)
    out += '"emit":['
    if on.callback:
        for emit in on.callback.emits:
            out += generate_emit_JSON(emit)
            out += ","
    if on.callback and on.callback.calls:
        for call in on.callback.calls:
            for emit in call.emits:
                out += generate_emit_JSON(emit)
                out += ","
    if out[-1] != "[":
        out = out[:-1]
    out += "],"
    out += '"location":{}'.format(generate_location_JSON(on.node.location))
    out += "}"
    return out


def generate_reactor_JSON(reactor):
    out = "{"
    out += '"name":"{}",'.format(reactor.get_name())
    out += '"on":['
    for method in reactor.methods:
        for on in method.ons:
            out += generate_on_JSON(on) + ","
    if out[-1] != "[":
        out = out[:-1]
    out += "],"
    out += '"location":{}'.format(generate_location_JSON(reactor.node.location))
    out += "}"
    return out


def generate_module_JSON(tree, name):
    out = "{"
    out += '"name":"{}",'.format(name)
    out += '"reactors":['
    for reactor in tree.reactors:
        out += generate_reactor_JSON(reactor)
        out += ","
    if out[-1] != "[":
        out = out[:-1]
    out += "]}"
    return out


def parse(module, files, outdir):
    print("Working on module", module)
    out = generate_module_JSON(analyse.create_tree(files, module), module)
    lock.acquire()
    try:
        with open(os.path.join(outdir, "_reactors" + ".json"), "a") as to_write:
            to_write.write(out + ",")
    finally:
        lock.release()


def initProcess(l):
    global lock
    lock = l


@run_on_docker
def register(command):
    command.help = "Generates documentation in JSON format for NUClear reactors and NUClear on and emit statements"

    command.add_argument(
        "--outdir",
        default="doc/extract-documentation/",
        help="The output directory. Default doc/extract-documentation/",
    )

    command.add_argument(
        "--indir", default="module", help="The root of the directories that you want to scan. Default: module."
    )

    command.add_argument(
        "-m",
        "--multiprocess",
        type=int,
        default=1,
        help="Enable multiprocessing for faster generation, uses a lot more RAM.",
    )


@run_on_docker
def run(outdir, indir, multiprocess, **kwargs):
    indir = os.path.abspath(indir)

    modules = {}

    src_dirs = []

    # Find all modules by walking the file tree
    for dirpath, dirnames, filenames in os.walk(indir):
        for dir in dirnames:
            if dir == "src":
                dirnames.remove("src")  # Don't look into the src directory
                src_dirs.append(os.path.join(dirpath, dir))

    # Find all the files in the source directories that aren't python files
    for src_dir in src_dirs:
        modules["/".join(src_dir.split("/")[0:-1])] = []
        for dirpath, dirnames, filenames in os.walk(src_dir):
            for filename in filenames:
                if os.path.splitext(filename)[1] == ".hpp" or os.path.splitext(filename)[1] == ".cpp":
                    modules["/".join(src_dir.split("/")[0:-1])].append(os.path.join(dirpath, filename))

    if multiprocess > 1:
        with open(os.path.join(outdir, "_reactors" + ".json"), "w") as to_write:
            to_write.write("[")
        lock = Lock()
        with Pool(min(cpu_count(), multiprocess), initializer=initProcess, initargs=(lock,)) as pool:
            pool.starmap(parse, zip(modules.keys(), modules.values(), repeat(outdir, len(modules))))

    else:
        with open(os.path.join(outdir, "_reactors" + ".json"), "w") as to_write:
            to_write.write("[")
            for module, files in modules.items():
                print("Working on module", module)
                to_write.write(generate_module_JSON(analyse.create_tree(files, module), module) + ",")

    with open(os.path.join(outdir, "_reactors" + ".json"), "rb+") as to_write:
        to_write.seek(-1, SEEK_END)
        to_write.write("]\n".encode("utf-8"))

    os.rename(os.path.join(outdir, "_reactors" + ".json"), os.path.join(outdir, "reactors" + ".json"))
