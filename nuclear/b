#!/usr/bin/python

import sys
import os
import argparse
import pkgutil

# Don't make .pyc files
sys.dont_write_bytecode = True

tools_path = os.path.dirname(os.path.realpath(__file__)) + os.sep + 'tools'

# Add our builtin tools to the path
sys.path.append(tools_path)

# Add add tools to the path in case the user has some in their pwd
sys.path.append('tools')
sys.path.append('nuclear/tools')

if __name__ == '__main__':

    # Root parser information
    command = argparse.ArgumentParser(description='This script is an optional helper script for performing common tasks for working with the NUClear roles system.')
    subcommands = command.add_subparsers(dest='command')
    subcommands.help = "The command to run from the script. See each help for more information."

    # Get all of the packages that are in the build tools
    print tools_path
    modules = pkgutil.iter_modules(path=[tools_path])

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

        # Associate our module_name with this tool
        tools[module_name] = tool

    # Parse our arguments
    args = command.parse_args()

    # Pass to our tool
    tools[args.command](**vars(args))
