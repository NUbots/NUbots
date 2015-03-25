#!/usr/bin/python

import docker

def register(command):

    # Set our commands help
    command.help='Manage roles in the codebase'

    # Add our subcommands
    subcommand = command.add_subparsers(dest='role_command')
    run_role = subcommand.add_parser('run', help='execute a compiled role in the system')
    run_role.add_argument('role', metavar='role', help='the name of the role to execute on the container')

def run(role='', **kwargs):
    print role


    # def compile(self):
    #     # Make our build folder if it doesn't exist
    #     if not os.path.exists('build'):
    #         print('Creating build folder...')
    #         os.mkdirs('build')

    #     # If we don't have an image, or it is out of date then we need to build one
    #     if not self._up_to_date():
    #         self.build()

    #     # If we don't have cmake built, run cmake from the docker container
    #     if not os.path.exists('build/build.ninja'):
    #         print('Running cmake...')
    #         self._docker_run('cmake', '..', '-GNinja', interactive=True)
    #         print('done')

    #     print('Running ninja...')
    #     self._docker_run('ninja', interactive=True)
    #     print('done')

    # def configure_compile(self):
    #     # Make our build folder if it doesn't exist
    #     if not os.path.exists('build'):
    #         print('Creating build folder...')
    #         os.mkdirs('build')

    #     # If we don't have an image, then we need to build one
    #     if not self._up_to_date():
    #         self.build()

    #     print('Running ccmake...')
    #     self._docker_run('ccmake', '..', '-GNinja', interactive=True)
    #     print('done')

    # def run_role(self, role):
    #     # If we don't have an image, then we need to build one
    #     if not self._up_to_date():
    #         self.build()

    #     print('Running {} on the container...'.format(role))
    #     self._docker_run('sh', '-c', '"bin/{}"'.format(role), interactive=True)
