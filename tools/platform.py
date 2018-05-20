#!/usr/bin/env python

import os
import b
import argparse
from termcolor import cprint
from subprocess import Popen


def get_toolchains():
    toolchains = {}

    # Work out what platforms are available
    for f in os.listdir(os.path.join(b.project_dir, 'Reel', 'toolchain')):
        if f.endswith('.cmake'):
            toolchains[f[:-6]] = os.path.join(b.project_dir, 'Reel', 'toolchain')

        elif os.path.isdir(os.path.join(b.project_dir, 'Reel', 'toolchain', f)):
            for g in os.listdir(os.path.join(b.project_dir, 'Reel', 'toolchain', f)):
                if g.endswith('.cmake'):
                    toolchains[g[:-6]] = os.path.join(b.project_dir, 'Reel', 'toolchain', g[:-6])

    return toolchains


def register(command):

    # Find toolchains
    toolchains = get_toolchains()

    # Install help
    command.help = 'Tools to work with building the codebase'

    # Module subcommands
    subcommands = command.add_subparsers(dest='workspace_command')

    select_comamnd = subcommands.add_parser('select', help='Choose a specific platform as the main platform')
    list_command = subcommands.add_parser('list', help='List all available platforms')
    build_command = subcommands.add_parser('build', help='Build currently selected platform')

    select_comamnd.add_argument(
        'platform',
        metavar='platform',
        choices=[t for t in toolchains],
        help='The platform to select as the primary workspace. One of <{}>'.format(', '.join(toolchains.keys()))
    )

    select_comamnd.add_argument(
        '-s', '--static', dest='static', action='store_true', help='perform a static build to get maximum performance'
    )

    select_comamnd.add_argument('cmake_args', nargs=argparse.REMAINDER, help='Extra arguments to pass to cmake')


def run(workspace_command, static=False, cmake_args=None, **kwargs):
    toolchains = get_toolchains()

    if workspace_command == 'list':
        print('Available toolchains:')
        print('\n'.join(['\t{}'.format(t) for t in toolchains]))

    elif workspace_command == 'select':
        platform = kwargs['platform']

        if platform not in toolchains:
            print(
                'Toolchain {} not available on this system. Please choose one of <{}>'.format(
                    ', '.join(toolchains.keys())
                )
            )

        else:
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
            symlink_success = True
            try:
                os.symlink(path, 'build')

            except OSError:
                symlink_success = False

            # Change to that directory
            os.chdir(path)

            # Build our default args
            args = [
                'cmake', b.project_dir, '-GNinja', '-DTOOLCHAIN_ROOT={}'.format(toolchains[platform]),
                '-DCMAKE_TOOLCHAIN_FILE={}'.format(os.path.join(toolchains[platform], '{}.cmake'.format(platform)))
            ]

            if static:
                args.append('-DSTATIC_LIBRARIES=ON')
                args.append('-DNUCLEAR_SHARED_BUILD=OFF')

            if cmake_args is not None:
                args += cmake_args

            # Run cmake
            env = os.environ
            env['PATH'] = os.pathsep.join([
                os.path.join(toolchains[platform], 'bin'),
                os.path.join(toolchains[platform], '..', 'bin'),
                env.get('PATH', '')
            ])

            # We should also be setting LD_LIBRARY_PATH here, but things get seriously broken.
            # env['LD_LIBRARY_PATH'] = os.path.join(toolchains[platform], 'lib')
            # So, for now, we need to set LD_LIBRARY_PATH externally.

            process = Popen(args=' '.join(args), shell=True, env=env)

            if process.wait() != 0:
                raise Exception('Failed to select platform "{}"'.format(platform))

            # Yell at windows users for having a crappy OS
            if not symlink_success:
                cprint(
                    'Windows does not support symlinks so we can\'t link to the build directory', 'red', attrs=['bold']
                )
                cprint(
                    'Instead you will need to change to the build_{} directory to build'.format(platform),
                    'red',
                    attrs=['bold']
                )

    elif workspace_command == 'build':
        if not os.path.exists(os.path.join(b.project_dir, 'build')):
            cprint('No platform is currently activated.', 'red', attrs=['bold'])
            cprint('Select a platform by running: ', 'red', end='', attrs=['bold'])
            print('./b platform select <platform name>.')
        else:
            # Change to build directory
            os.chdir(os.path.join(b.project_dir, 'build'))

            platform = None
            for toolchain in toolchains.keys():
                if os.path.realpath(os.path.join(b.project_dir, 'build')).endswith(toolchain):
                    platform = toolchain

            if platform is None:
                cprint('An invalid platform is currently activated.', 'red', attrs=['bold'])
                cprint('Select a valid platform by running: ', 'red', end='', attrs=['bold'])
                print('./b platform select <platform name>.')

            # Run ninja
            env = os.environ
            env['PATH'] = os.pathsep.join([
                os.path.join(toolchains[platform], 'bin'),
                os.path.join(toolchains[platform], '..', 'bin'),
                env.get('PATH', '')
            ])

            # We should also be setting LD_LIBRARY_PATH here, but things get seriously broken.
            # env['LD_LIBRARY_PATH'] = os.path.join(toolchains[platform], 'lib')
            # So, for now, we need to set LD_LIBRARY_PATH externally.

            process = Popen(
                args='cd {} && ninja; cd {}'.format(os.path.join(b.project_dir, 'build'), b.project_dir),
                shell=True,
                env=env
            )

            if process.wait() != 0:
                raise Exception('Failed to build {}'.format(platform))
