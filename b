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
    command = argparse.ArgumentParser(description='This script is an optional helper script for performing common tasks related to building and running the NUbots code and related projects.')
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

    # # Compile subcommand
    # compile_command = subcommands.add_parser('compile', help='Compile the NUbots source code')
    # compile_command.add_argument('-c', '--configure', help='Configure the options for the compilation (ccmake)', action='store_true')

    # # Role subcommand
    # role_command = subcommands.add_parser('role', help='Manage roles in the codebase')
    # role_subcommand = role_command.add_subparsers(dest='role_command')
    # run_role = role_subcommand.add_parser('run', help='execute a compiled role in the system')
    # run_role.add_argument('role', metavar='role', help='the name of the role to execute on the container')

    # # Parse our command line arguments
    # args = command.parse_args()
    # if args.subcommand == 'docker':
    #     Docker(**vars(args)).run()
    # elif args.subcommand == 'compile':
    #     if args.configure:
    #         Docker().configure_compile()
    #     else:
    #         Docker().compile()
    # elif args.subcommand == 'role':
    #     if args.role_command == 'run':
    #         Docker().run_role(args.role)
    # elif args.subcommand == 'module':
    #     if args.module_command == 'generate':
    #         tools.module.Module(**vars(args)).run()
    #         # TODO do module generate
