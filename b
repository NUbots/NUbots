#!/usr/bin/python

import sys
import os
import shutil
from subprocess import call

numArgs = len(sys.argv)

# print 'Number of arguments:', numArgs, 'arguments.'
# print 'Argument List:', str(sys.argv)

if numArgs == 1:
    # Print command summary?
    print """
Usage: b <command> [arguments]

NUbots Helper
----------------------------------------------------------------------
This script is an optional helper script for performing common tasks
related to building and running NUClearPort and related projects.

Command summary:
  - clean       Deletes the build directory.
  - cmake       Runs cmake in the build directory (creating it if it doesn't exist).
  - make        Runs make in the build directory (creates it and runs cmake if it doesn't exist).
  - makej       Same as make, but runs 'make -j'.
  - run <role>  Runs the binary for the role of the given name.
  - create_box	Builds the nubots Vagrant box using Packer.
                (the box is first deleted if it already exists)
"""

elif sys.argv[1] == 'clean':
    if os.path.exists('build'):
        shutil.rmtree('build')

elif sys.argv[1] == 'cmake':
    if not os.path.exists('build'):
        os.mkdir('build') 
    call(['cmake', '..'], cwd='build')

elif sys.argv[1] == 'make' or sys.argv[1] == 'makej':
    if not os.path.exists('build'):
        os.mkdir('build') 
        call(['cmake', '..'], cwd='build')

    if sys.argv[1] == 'make':
        call('make', cwd='build')
    else:
        call(['make', '-j'], cwd='build')

elif sys.argv[1] == 'run':
    if numArgs < 3:
	print '''
Usage: b run <role>

Please provide the name of the role to run.
'''
    elif os.path.isfile("build/roles/{}".format(sys.argv[2])):
        call("./roles/{}".format(sys.argv[2]), cwd='build/')
    else:
        print "The role '{}' does not exist or did not build correctly.".format(sys.argv[2])
elif sys.argv[1] == 'create_box':
  if os.path.isfile("packer/nubots-ubuntu-12-04-x86-virtualbox.box"):
    call(['rm', 'nubots-ubuntu-12-04-x86-virtualbox.box'], cwd='packer')
  call(['packer', 'build', '-only=virtualbox-iso', 'template.json'], cwd='packer')
  call(['vagrant', 'box', 'remove', 'nubots-14.02'])
  call(['vagrant', 'box', 'add', 'nubots-14.02', 'nubots-ubuntu-12-04-x86-virtualbox.box'], cwd='packer')
