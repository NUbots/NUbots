#!/usr/bin/python

import os
import sys
import argparse
import shutil
import platform
import subprocess


def which(program):
    import os
    def is_exe(fpath):
        return os.path.isfile(fpath) and os.access(fpath, os.X_OK)

    fpath, fname = os.path.split(program)
    if fpath:
        if is_exe(program):
            return program
    else:
        for path in os.environ["PATH"].split(os.pathsep):
            path = path.strip('"')
            exe_file = os.path.join(path, program)
            if is_exe(exe_file):
                return exe_file

    return None

class Docker():
    def __init__(self, docker_command, **kwargs):
        # Check that docker is installed
        self._check_docker()

        # Setup our docker environment
        self._setup_docker_environment()

        self.command = getattr(self, docker_command)

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
                if status == 'poweroff':
                    print('Powering on Boot2Docker VM...')
                    result = subprocess.call(['boot2docker', 'up'])

                    # Check for errors while booting
                    if result != 0:
                        print('There was an error while initializing the boot2docker vm (see error above)')
                        sys.exit(1)

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
            print "Docker is not installed, please download and install it"
            sys.exit(1)

        # Check that either we are native linux, or we have a vm provider (docker machine or boot2docker)
        if not (platform.system() == 'Linux' or which('docker-machine') or which('boot2docker')):
            print "The requirements for the docker daemon are not met."
            print "You must either be running on linux, or have a Virtual Machine provider for docker (either docker-machine or boot2docker)"

    def build(self):
        # See if the NUbots image is there
        image = subprocess.check_output(['docker', 'images', '-q', 'nubots/nubots'])

        # If we don't have an image, then build it
        if not image:
            subprocess.call(['docker', 'build', '-t=nubots/nubots', '.'])

    def clean(self):
        print 'TODO CLEAN'

    def rebuild(self):
        # Clean
        self.clean()
        # and rebuild
        self.build()

    def run(self):
        self.command()
        pass

    def _compile(self, arguments):
        if not os.path.exists('build'):
            print('Creating build folder...')
            os.mkdir('build')

        if not os.path.exists('build/CMakeCache.txt'):
            print('Running cmake...')
            subprocess.call(['cmake', '..', '-G', 'Ninja'], cwd='build')
            print('done')

        print('Running ninja...')
        subprocess.call(['ninja'], cwd='build')
        print('done')

if __name__ == "__main__":

    # Root parser information
    command = argparse.ArgumentParser(description='This script is an optional helper script for performing common tasks related to building and running the NUbots code and related projects.')
    subcommands = command.add_subparsers(dest='subcommand')

    # Docker subcommand
    docker_command = subcommands.add_parser('docker', help='Manage the docker container used to build')
    docker_subcommands = docker_command.add_subparsers(dest='docker_command')
    docker_subcommands.add_parser('build', help='Build the docker image used to build the code and spin up any required Virtual Machines')
    docker_subcommands.add_parser('clean', help='Delete the built docker image so that it will have to be rebuilt')
    docker_subcommands.add_parser('rebuild', help='Delete the built docker image and rebuild it')

    # Compile subcommand
    compile_command = subcommands.add_parser('compile', help='Compile the NUbots source code')

    # Run subcommand
    run_command = subcommands.add_parser('run', help='Run a compiled role on the docker VM')

    # Module subcommand
    module_command = subcommands.add_parser('module', help='Manage NUClear modules in the codebase')
    module_subcommands = module_command.add_subparsers()

    # Generate module subcommand
    module_generate_command = module_subcommands.add_parser('generate', help='Generate a new NUClear module based on a template')
    module_generate_command.add_argument('path', metavar='path', help='a path to the new module (from the modules directory)')

    # Parse our command line arguments
    args = vars(command.parse_args())

    if args['subcommand'] == 'docker':
        Docker(**args).run()
