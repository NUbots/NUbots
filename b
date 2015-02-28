#!/usr/bin/python

import os
import sys
import shutil
import platform
import subprocess

class Builder():
    def print_command_summary(self):
        print """
Usage: b <command> [arguments]

NUbots Helper
----------------------------------------------------------------------
This script is an optional helper script for performing common tasks
related to building and running the NUbots code and related projects.

Command summary:
  - help              Show this help.
  - clean             Deletes the build directory.
  - destroy           Deletes any existing Boot2Docker VM
  - build             Build the code, this will automatically create a Boot2Docker VM and container if they do not exist
"""

    def is_docker_native(self):
        # True if docker can be run natively, currently this support is only on Linux
        return True if platform.system() =='Linux' else False

    def boot2docker_status(self):
        try:
            status = subprocess.check_output(['boot2docker', 'status'], stderr=subprocess.STDOUT)
            return (True, status.strip())
        except subprocess.CalledProcessError:
            return (False, None)

    def check_build_requirements(self):
        # Set current working directory to this script's location
        abspath = os.path.abspath(__file__)
        dname = os.path.dirname(abspath)
        os.chdir(dname)

        if self.is_docker_native():
            # TODO: check if docker is installed
            return True
        else:
            # TODO: check if boot2docker is installed
            print('Checking for Boot2Docker VM...'),
            (exists, status) = self.boot2docker_status()
            if exists:
                print('yes.')
            if not exists:
                print('no.')
                print('Initializing Boot2Docker VM...'),
                subprocess.call(['boot2docker', 'init'])
                print('done.')
                (exists, status) = self.boot2docker_status()

            if not exists:
                print('Error Initializing.')
                return False
            elif status == 'poweroff':
                self.up(None)
                (exists, status) = self.boot2docker_status()

            if status == 'running':
                return True

        return False

    def up(self, arguments):
        print('Starting Boot2Docker VM...')
        abspath = os.path.abspath(__file__)
        dname = os.path.dirname(abspath)
        if 'cygwin' in platform.system().lower():
            dname = subprocess.check_output(['cygpath', '-w', dname]).strip()
            dname = dname.encode('string_escape')
        subprocess.call(['boot2docker', 'up', '--vbox-share={}=nubots'.format(dname)])
        print('Mounting shares...'),
        subprocess.call(['boot2docker', 'ssh', 'sudo mkdir -p /nubots && sudo mount -t vboxsf nubots /nubots'])
        print('done')

    def down(self, arguments):
        subprocess.call(['boot2docker', 'down'])

    def destroy(self, arguments):
        subprocess.call(['boot2docker', 'destroy'])

    def clean(self, arguments):
        if os.path.exists('build'):
            shutil.rmtree('build')

    def build(self, arguments):
        if not self.check_build_requirements():
            return False;

        print('Checking for NUbots image...'),
        layer = self.execute_docker_command('images', '-q', 'nubots/nubots', get_output=True)
        if layer:
            print('yes.')
        else:
            print('no.')
            print('Creating NUbots image...')
            self.execute_docker_command('build', '-t=nubots/nubots', '.')
            print('done.'),
        print('Creating build folder...')
        self.execute_docker_command('run', '-t', '-P=false', '-v', '/nubots:/nubots/NUbots', 'nubots/nubots', 'mkdir build')
        print('done')
        print('Running cmake...')
        self.execute_docker_command('run', '-t', '-P=false', '-v', '/nubots:/nubots/NUbots', 'nubots/nubots', '"cd build && cmake .. -GNinja"')
        print('done')
        print('Running ninja...')
        self.execute_docker_command('run', '-t', '-P=false', '-v', '/nubots:/nubots/NUbots', 'nubots/nubots', '"cd build && ninja"')
        print('done')


    def execute_docker_command(self, command, *arguments, **kwargs):

        docker_command = ['docker', ' '.join([command] + list(arguments))]

        if self.is_docker_native():
            subprocess.call(docker_command)
        else:
            boot2docker_command = ['boot2docker', 'ssh', ' '.join(['cd', '/nubots', '&&'] + docker_command)]

            if kwargs.get('get_output', False):
                print('Running: {}'.format(boot2docker_command))
                return subprocess.check_output(boot2docker_command)
            else:
                subprocess.call(boot2docker_command)

    def parse_arguments(self, argv):
        command = ''
        arguments = []

        if len(sys.argv) >= 2:
            command = sys.argv[1]
        if len(sys.argv) >= 3:
            arguments = sys.argv[2:]

        self.parse_command(command, arguments)

    def parse_command(self, command, arguments):
        arg0 = ''
        if len(arguments) >= 1:
            arg0 = arguments[0]

        if (command == '' or
            command == 'help' or
            command == '--help'):
            self.print_command_summary()

        elif command == 'clean':
            self.clean(arguments)

        elif command == 'up':
            self.up(arguments)

        elif command == 'down':
            self.down(arguments)

        elif command == 'destroy':
            self.destroy(arguments)

        elif command == 'build':
            self.build(arguments)
        else:
            print 'Unknown Command: {}'.format(command)


if __name__ == "__main__":

    b = Builder();
    b.parse_arguments(sys.argv)
