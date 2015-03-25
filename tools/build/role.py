#!/usr/bin/python

import docker

def register(command):

    # Set our commands help
    command.help = 'Manage roles in the codebase'

    # Add our subcommands
    subcommand = command.add_subparsers(dest='role_command')
    run_role = subcommand.add_parser('run', help='execute a compiled role in the system')
    run_role.add_argument('role', metavar='role', help='the name of the role to execute on the container')

def run(role='', **kwargs):
    # Get a docker instance
    docker = Docker()

    print('Running {} on the container...'.format(role))
    self._docker_run('sh', '-c', '"bin/{}"'.format(role), interactive=True)
