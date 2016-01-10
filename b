#!/usr/bin/python

import sys
import argparse
import pkgutil

# Don't make .pyc files
sys.dont_write_bytecode = True

# Add tools/build to our python path
sys.path.append('tools/build')

if __name__ == '__main__':

    # Root parser information
    command = argparse.ArgumentParser(description='This script is an optional helper script for performing common tasks for working with the NUClear roles system.')
    subcommands = command.add_subparsers(dest='command')

    # Get all of the packages that are in the build tools
    modules = pkgutil.iter_modules(path=['tools/build'])

    # Our tools dictionary
    tools = {}

    # Loop through all the modules we have
    for loader, module_name, ispkg in modules:

        # Get our module, class name and registration function
        module = loader.find_module(module_name).load_module(module_name)
        tool = getattr(module, 'run')
        register = getattr(module, 'register')

        # Let the tool register it's arguments
        register(subcommands.add_parser(module_name))

        # associate our module_name with this tool
        tools[module_name] = tool

        # Call our register function with the subcommands thing

        # What the register function returns will be the "subcommands" to forward to it

    # Parse our arguments
    args = command.parse_args()

    # Pass to our tool
    tools[args.command](**vars(args))
