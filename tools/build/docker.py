#!/usr/bin/python

import sys
import os
import platform
import subprocess

def register(command):

    # Set our commands help
    command.help = 'Manage the docker container used to build'

    # Docker subcommands
    subcommands = command.add_subparsers(dest='docker_command')
    subcommands.add_parser('build', help='Build the docker image used to build the code and spin up any required Virtual Machines')
    subcommands.add_parser('clean', help='Delete the built docker image so that it will have to be rebuilt')
    subcommands.add_parser('rebuild', help='Delete the built docker image and rebuild it')
    subcommands.add_parser('shell', help='Get a non persistant interactive shell on the container')

def run(docker_command='', **kwargs):
    # Setup docker
    docker = Docker()

    # Run the command
    getattr(docker, docker_command)()

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
        # Add Docker binary to end of PATH (for windows)
        os.environ['PATH'] = os.environ['PATH'] + os.pathsep + os.getcwd() + '/tools/bin'

        # Check that docker is installed
        self._check_docker()

        # Setup our docker environment
        self._setup_docker_environment()

        # Ensure we have a build folder
        if not os.path.exists('build'):
            print('Creating build folder...')
            os.mkdir('build')

        # Get our image provided to us or use a default
        docker_image = kwargs.get('docker_image', 'darwin')

        # Set relevant sections for our docker image
        self.image = {
            'path': 'tools/images/{}'.format(docker_image),
            'tag':  docker_image.replace('/', '_'),
            'name': 'nubots/nubots:{}'.format(docker_image.replace('/', '_'))
        }

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

    def run(self, *args, **kwargs):
        # Ensure we are updated
        if not self._up_to_date():
            self.build()

        # Pass them on to our internal run
        self._docker_run(*args, **kwargs)

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
