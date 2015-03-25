#!/usr/bin/python

import os
from docker import Docker

def register(command):

    # Set our commands help
    command.help = 'Compile the NUbots source code'

    # We have a configure command
    command.add_argument('-c', '--configure', help='Configure the options for the compilation (ccmake)', action='store_true')

def run(configure=False, **kwargs):

    # Get a docker instance
    docker = Docker()

    if configure:
        # Run ccmake using docker
        docker.run('ccmake', '..', '-GNinja', interactive=True)
    else:
        if not os.path.exists('build/build.ninja'):
            print('Running cmake...')
            docker.run('cmake', '..', '-GNinja', interactive=True)
            print('done')

        docker.run('ninja', interactive=True)
