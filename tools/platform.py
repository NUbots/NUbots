#!/usr/bin/env python

import os
import b
from subprocess import call, STDOUT

def register(command):

    # Install help
    command.help = 'Tools to work with building the codebase'

    # Module subcommands
    subcommands = command.add_subparsers(dest='workspace_command')

    init_command = subcommands.add_parser('select', help='Choose a specific platform as the main platform')

    toolchains = []
    # Work out what platforms are available
    for f in os.listdir('/nubots/toolchain/'):
        if f.endswith('.cmake'):
            toolchains.append(f[:-6])

    init_command.add_argument('platform', metavar='platform', choices=toolchains, help='the platform to select as the primary workspace')

def run(workspace_command, **kwargs):
    if workspace_command == 'select':
        platform = kwargs['platform']

        print('Activating workspace as platform {}'.format(platform))
        path = os.path.join(b.project_dir, 'build_{}'.format(platform))

        # Make the directory
        try:
            os.makedirs(path)
        except FileExistsError:
            pass

        # Make the symlink to make this the main directory
        try:
            os.remove('build')
        except FileNotFoundError:
            pass

        # Try to make the symlink, this will fail if you use windows
        try:
            os.symlink(path, 'build')
        except OSError:
            print("Windows does not support symlinks, you will need to cd to build_{}".format(platform))

        # Change to that directory
        os.chdir(path)

        # Run cmake
        call(['cmake', b.project_dir, '-GNinja', '-DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/{}.cmake'.format(platform)])


