#!/usr/bin/python

import os
import sys
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

class Builder():
    def __init__(self):
        # Make sure we have docker installed
        if not which("docker"):
            print "docker is not installed"
            sys.exit(1);

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
        return True if platform.system() == 'Linux' else False

    def boot2docker_status(self):
        try:
            status = subprocess.check_output(['boot2docker', 'status'], stderr=subprocess.STDOUT)
            return (True, status.strip())
        except subprocess.CalledProcessError:
            return (False, None)

    def set_docker_environment(self):
        # Do we need to find our docker host? (has it already been set externally?)
        if 'DOCKER_HOST' not in os.environ:
            # Are we not native? otherwise we don't need to do anything
            if not self.is_docker_native():

                # Check if docker-machine is installed (prefered)
                if which('docker-machine'):
                    pass

                # Check if boot2docker is installed
                elif which('boot2docker'):
                    # Get boot2docker's status
                    (exists, status) = self.boot2docker_status()

                    # If there isn't a boot2docker vm yet
                    if not exists:
                        print('Initializing Boot2Docker VM...')
                        result = subprocess.call(['boot2docker', 'init'])

                        # Check for errors while setting up
                        if result != 0:
                            print('There was an error while initializing the boot2docker vm (see error above)')
                            sys.exit(1)

                        print('Done.')

                        (exists, status) = self.boot2docker_status()

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

                else:
                    print("There is not a suitable way to run Docker containers, install docker-machine or boot2docker (or use linux as your os)")
                    sys.exit(1)

    def fun(self):
        # See if the NUbots image is there
        image = subprocess.check_output(['docker', 'images', '-q', 'nubots/nubots'])

        if not image:
            subprocess.call(['docker', 'build', '-t=nubots/nubots', '.'])


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
        subprocess.call(['boot2docker', 'ssh', 'sudo mkdir -p /nubots/NUbots && sudo mount -t vboxsf nubots /nubots/NUbots'])
        subprocess.call(['boot2docker', 'ssh', 'cp /nubots/NUbots/dockershell /home/docker/dockershell'])
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
        self.execute_docker_command('run', '-t', '-v', '/nubots/NUbots:/nubots/NUbots', 'nubots/nubots', './b compile')

        if not self.is_docker_native():
            ip = subprocess.check_output(['boot2docker', 'ip'])
            print
            print('To access the built files, you can SSH to docker:tcuser@{} and run ./dockershell'.format(ip))
            print

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

    def execute_docker_command(self, command, *arguments, **kwargs):

        docker_command = ['docker', ' '.join([command] + list(arguments))]

        if self.is_docker_native():
            subprocess.call(docker_command)
        else:
            boot2docker_command = ['boot2docker', 'ssh', ' '.join(['cd', '/nubots/NUbots', '&&'] + docker_command)]

            if kwargs.get('get_output', False):
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

        elif command == 'compile':
            self._compile(arguments)
        else:
            print 'Unknown Command: {}'.format(command)


if __name__ == "__main__":

    b = Builder();
    b.set_docker_environment()
    b.fun()
    # b.parse_arguments(sys.argv)
