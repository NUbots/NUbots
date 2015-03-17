#!/usr/bin/python

import os
import sys
import argparse
import shutil
import platform
import subprocess

def which(program):

    # If we are on windows we might need .exe on the end
    if platform.system() == 'Windows' and program[-3:] != '.exe':
        program += '.exe'

    def is_executable(fpath):
        return os.path.isfile(fpath) and os.access(fpath, os.X_OK)

    fpath, fname = os.path.split(program)
    if fpath:
        if is_executable(program):
            return program
    else:
        for path in os.environ['PATH'].split(os.pathsep):
            path = path.strip('"')
            exe_file = os.path.join(path, program)
            if is_executable(exe_file):
                return exe_file

    return None

class Docker():
    def __init__(self, **kwargs):
        # Check that docker is installed
        self._check_docker()

        # Setup our docker environment
        self._setup_docker_environment()

        # Get our image provided to us or use a default
        docker_image = kwargs.get('docker_image', 'darwin')

        # Set relevant sections for our docker image
        self.image = {
            'path': 'tools/images/{}'.format(docker_image),
            'tag':  docker_image.replace('/', '_'),
            'name': 'nubots/nubots:{}'.format(docker_image.replace('/', '_'))
        }

        # If we have a command to run then set it up to run
        if 'docker_command' in kwargs:
            self.command = getattr(self, kwargs['docker_command'])

    def _up_to_date(self):
        # Check we have an image
        if not self.image['tag'] in subprocess.check_output(['docker', 'images', 'nubots/nubots']):
            return False

        # Get our timestamps of our Dockerfile
        x = os.path.getmtime(self.image['path'])
        # Get the timestamp of the build
        y = int(self._docker_run('cat', '/container_built_at'))

        # Make sure the build is newer
        return x < y

    def _share_path(self):
        # Get the path to the b script
        abspath = os.path.abspath(__file__)
        local_name = os.path.dirname(abspath)
        remote_name = local_name

        # For cygwin we need to convert our path to a windows path
        if 'cygwin' in platform.system().lower():
            local_name = subprocess.check_output(['cygpath', '-w', local_name]).strip()
            local_name = local_name.encode('string_escape')

        # For Windows we need to escape our path
        elif platform.system() == 'Windows':
            remote_name = '/nubots/NUbots'

        # For other platforms the paths are the same
        return (local_name, remote_name)

    def _docker_run(self, *args, **kwargs):
        command = (['docker'
            , 'run'
            , '--publish=12000:12000'
            , '--publish=12001:12001']
            + (['-t', '-i'] if kwargs.get('interactive', False) else [])
            + (['-d'] if kwargs.get('detached', False) else [])
            + [ '-v'
            , '{}:/nubots/NUbots'.format(self._share_path()[1])
            , self.image['name']] + list(args))

        # If we're not detached then run
        if 'interactive' in kwargs:
            subprocess.call(command)

        # If we're detached get our argument
        elif 'detached' in kwargs:
            return subprocess.check_output(command)

        # Otherwise just run
        else:
            return subprocess.check_output(command)

    def _boot2docker_status(self):
        try:
            status = subprocess.check_output(['boot2docker', 'status'], stderr=subprocess.STDOUT)
            return (True, status.strip())
        except subprocess.CalledProcessError:
            return (False, None)

    def _setup_docker_environment(self):
        # Do we need to do any configuration for our docker environment
        # If we are linux or have a host hardcoded we don't need to do anything
        if 'DOCKER_HOST' not in os.environ and platform.system() != 'Linux':

            # Check if docker-machine is installed (prefered)
            if which('docker-machine'):
                pass

            # Check if boot2docker is installed
            elif which('boot2docker'):
                # Get boot2docker's status
                (exists, status) = self._boot2docker_status()

                # If there isn't a boot2docker vm yet
                if not exists:
                    print('Initializing Boot2Docker VM...')
                    result = subprocess.call(['boot2docker', 'init'])

                    # Check for errors while setting up
                    if result != 0:
                        print('There was an error while initializing the boot2docker vm (see error above)')
                        sys.exit(1)

                    print('Done.')

                    (exists, status) = self._boot2docker_status()

                # If the boot2docker vm is powered off
                if status != 'running':
                    print('Powering on Boot2Docker VM...')

                    # Work out the local path to our shared folder
                    share_paths = self._share_path()

                    # Startup our VM
                    result = subprocess.call(['boot2docker', 'up', '--vbox-share={}=nubots'.format(share_paths[0])])

                    # Check for errors while booting
                    if result != 0:
                        print('There was an error while initializing the boot2docker vm (see error above)')
                        sys.exit(1)

                    print('Mounting shares...'),
                    subprocess.call(['boot2docker', 'ssh', 'sudo mkdir -p {0} && sudo mount -t vboxsf nubots {0}'.format(share_paths[1])])

                    print('Done.')

                # Now we can get the environment variables we need
                process = subprocess.Popen(['boot2docker', 'shellinit'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                exports = process.communicate()[0]
                result = process.wait()
                exports = [e.strip()[7:].split('=') for e in exports.strip().split('\n')]

                # Set our enviroment variables
                for e in exports:
                    os.environ[e[0]] = e[1]

    def _check_docker(self):
        # Check that docker is installed
        if not which('docker'):
            print('Docker is not installed, please download and install it')
            sys.exit(1)

        # Check that either we are native linux, or we have a vm provider (docker machine or boot2docker)
        if not (platform.system() == 'Linux' or which('docker-machine') or which('boot2docker')):
            print('The requirements for the docker daemon are not met.')
            print('You must either be running on linux, or have a Virtual Machine provider for docker (either docker-machine or boot2docker)')

    def build(self):
        # Get docker to build our vm
        subprocess.call(['docker', 'build', '-t={}'.format(self.image['name']), self.image['path']])

        # Run an extra command to timestamp when this was built (for checking with compile)
        container = self._docker_run('sh', '-c', 'date +"%s" > /container_built_at', detached=True).strip()
        subprocess.check_output(['docker', 'wait', container])
        subprocess.check_output(['docker', 'commit', container, self.image['name']])

    def clean(self):
        print('TODO CLEAN')

    def rebuild(self):
        # Clean
        self.clean()
        # and rebuild
        self.build()

    def shell(self):
        # Run a docker command that will give us an interactive shell
        self._docker_run('/bin/bash', interactive=True)

    def compile(self):
        # Make our build folder if it doesn't exist
        if not os.path.exists('build'):
            print('Creating build folder...')
            os.mkdir('build')

        # If we don't have an image, or it is out of date then we need to build one
        if not self._up_to_date():
            self.build()

        # If we don't have cmake built, run cmake from the docker container
        if not os.path.exists('build/build.ninja'):
            print('Running cmake...')
            self._docker_run('cmake', '..', '-GNinja', interactive=True)
            print('done')

        print('Running ninja...')
        self._docker_run('ninja', interactive=True)
        print('done')

    def configure_compile(self):
        # If we don't have an image, then we need to build one
        if not self._up_to_date():
            self.build()

        # Make our build folder if it doesn't exist
        if not os.path.exists('build'):
            print('Creating build folder...')
            os.mkdir('build')

        print('Running ccmake...')
        self._docker_run('ccmake', '..', '-GNinja', interactive=True)
        print('done')

    def run_role(self, role):
        # If we don't have an image, then we need to build one
        if not self._up_to_date():
            self.build()

        print('Running {} on the container...'.format(role))
        self._docker_run('bin/{}'.format(role), interactive=True)
        print('done')

    def run(self):
        self.command()

if __name__ == '__main__':

    # Add Docker binary to end of PATH (for windows)
    os.environ['PATH'] = os.environ['PATH'] + os.pathsep + os.path.dirname(os.path.abspath(__file__)) + '/tools/bin'

    # Root parser information
    command = argparse.ArgumentParser(description='This script is an optional helper script for performing common tasks related to building and running the NUbots code and related projects.')
    subcommands = command.add_subparsers(dest='subcommand')

    # Docker subcommand
    docker_command = subcommands.add_parser('docker', help='Manage the docker container used to build')
    docker_subcommands = docker_command.add_subparsers(dest='docker_command')
    docker_subcommands.add_parser('build', help='Build the docker image used to build the code and spin up any required Virtual Machines')
    docker_subcommands.add_parser('clean', help='Delete the built docker image so that it will have to be rebuilt')
    docker_subcommands.add_parser('rebuild', help='Delete the built docker image and rebuild it')
    docker_subcommands.add_parser('shell', help='Get a non persistant interactive shell on the container')

    # Compile subcommand
    compile_command = subcommands.add_parser('compile', help='Compile the NUbots source code')
    compile_command.add_argument('-c', '--configure', help='Configure the options for the compilation (ccmake)', action='store_true')

    # Role subcommand
    role_command = subcommands.add_parser('role', help='Manage roles in the codebase')
    role_subcommand = role_command.add_subparsers(dest='role_command')
    run_role = role_subcommand.add_parser('run', help='execute a compiled role in the system')
    run_role.add_argument('role', metavar='role', help='the name of the role to execute on the container')

    # Module subcommand
    module_command = subcommands.add_parser('module', help='Manage NUClear modules in the codebase')
    module_subcommands = module_command.add_subparsers()

    # Generate module subcommand
    module_generate_command = module_subcommands.add_parser('generate', help='Generate a new NUClear module based on a template')
    module_generate_command.add_argument('path', metavar='path', help='a path to the new module (from the modules directory)')

    # Parse our command line arguments
    args = command.parse_args()

    if args.subcommand == 'docker':
        Docker(**vars(args)).run()
    elif args.subcommand == 'compile':
        if args.configure:
            Docker().configure_compile()
        else:
            Docker().compile()
    elif args.subcommand == 'role':
        if args.role_command == 'run':
            Docker().run_role(args.role)
