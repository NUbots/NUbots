#!/usr/bin/python3

import os
from io import SEEK_END
from itertools import repeat
from multiprocessing import Lock, Pool, cpu_count

import analyse
import b
from dockerise import run_on_docker


def generate_location_JSON(location):
    """
    Generates a JSON representation for a code location

    Args:
        location (clang.cindex.SourceLocation): A location in a translation unit

    Returns:
        str: The JSON representation of the code location
    """
    out = "{"
    out += '"file":"{}",'.format(location.file)
    out += '"line":"{}",'.format(location.line)
    out += '"column":"{}",'.format(location.column)
    out += '"offset":"{}"'.format(location.offset)
    out += "}"
    return out


def generate_emit_JSON(emit):
    """
    Generates JSON for a NUClear emit statement

    Args:
        emit (analyse.struct.Emit): A structure with information about an emit statement

    Returns:
        str: The JSON representation of the emit statement information
    """
    out = "{"
    out += '"scope":"{}",'.format(emit.scope)
    out += '"type":"{}",'.format(emit.type)
    out += '"location":{}'.format(generate_location_JSON(emit.node.location))
    out += "}"
    return out


def generate_on_JSON(on):
    """
    Generates JSON for a NUClear on statement

    Args:
        on (analyse.struct.On): A structure with information about an on statement

    Returns:
        str: The JSON representation of the on statement information
    """
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
    """
    Generates JSON for a NUClear reactor statement

    Args:
        on (analyse.struct.Reactor): A structure with information about a reactor statement

    Returns:
        str: The JSON representation of the reactor statement information
    """
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


def generate_module_JSON(module, name):
    """
    Generates JSON for a NUClearRoles Module

    Args:
        on (analyse.struct.Module): A structure with information about a Module

    Returns:
        str: The JSON representation of the Module information
    """
    out = "{"
    out += '"name":"{}",'.format(name)
    out += '"reactors":['
    for reactor in module.reactors:
        out += generate_reactor_JSON(reactor)
        out += ","
    if out[-1] != "[":
        out = out[:-1]
    out += "]}"
    return out


def parse(module, files, outdir):
    """
    Generates JSON for a module and writes it to the file _reactors.json

    This also uses the lock that is passed into the process at initProcess to stop concurrent writes to the file

    Args:
        module (str): Module directory
        files (list[str]): A list of files in the module directory to parse
        outdir (str): The path to the directory to write the file _reactors.json to
    """
    print("Working on module", module)
    out = generate_module_JSON(analyse.create_tree(files, module), module)
    lock.acquire()
    try:
        with open(os.path.join(outdir, "_reactors" + ".json"), "a") as to_write:
            to_write.write(out + ",")
    finally:
        lock.release()


def initProcess(l):
    """
    Stores the lock globally in the process

    Args:
        l (Lock): A lock for writing the output file
    """
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

    src_dirs = []

    # Find all modules by walking the file tree
    for dirpath, dirnames, filenames in os.walk(indir):
        for dir in dirnames:
            if dir == "src":
                dirnames.remove("src")  # Don't look into the src directory
                src_dirs.append(os.path.join(dirpath, dir))

    modules = {}

    # Find all the files in the source directories that aren't python files
    for src_dir in src_dirs:
        modules["/".join(src_dir.split("/")[0:-1])] = []
        for dirpath, dirnames, filenames in os.walk(src_dir):
            for filename in filenames:
                if os.path.splitext(filename)[1] == ".hpp" or os.path.splitext(filename)[1] == ".cpp":
                    modules["/".join(src_dir.split("/")[0:-1])].append(os.path.join(dirpath, filename))

    # Create the file, start the JSON array and truncate.
    with open(os.path.join(outdir, "_reactors" + ".json"), "w") as to_write:
        to_write.write("[")

    # If we are multiprocessing, make a pool to process in parallel. The size of the pool is the minimum of value passed
    # in or the cpu thread count
    if multiprocess > 1:
        lock = Lock()
        with Pool(min(cpu_count(), multiprocess), initializer=initProcess, initargs=(lock,)) as pool:
            pool.starmap(parse, zip(modules.keys(), modules.values(), repeat(outdir, len(modules))))
    # We are not multiprocessing, so we just do everything on the current process
    else:
        with open(os.path.join(outdir, "_reactors" + ".json"), "a") as to_write:
            for module, files in modules.items():
                print("Working on module", module)
                to_write.write(generate_module_JSON(analyse.create_tree(files, module), module) + ",")

    # Remove the last comma and end the JSON list
    with open(os.path.join(outdir, "_reactors" + ".json"), "rb+") as to_write:
        to_write.seek(-1, SEEK_END)
        to_write.write("]\n".encode("utf-8"))

    # Rename the file to remove the underscore
    os.rename(os.path.join(outdir, "_reactors" + ".json"), os.path.join(outdir, "reactors" + ".json"))
