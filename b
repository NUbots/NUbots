#!/usr/bin/python

import sys
import os
import shutil
from subprocess import call

numArgs = len(sys.argv)

if numArgs == 1:
    # Print command summary?
    print """
Usage: b <command> [arguments]

NUbots Helper
----------------------------------------------------------------------
This script is an optional helper script for performing common tasks
related to building and running NUClearPort and related projects.

Command summary:
  - clean	Deletes the build directory.
  - cmake	Runs cmake in the build directory (creating it if it doesn't exist).
  - make	Runs make in the build directory (creates it and runs cmake if it doesn't exist).
  - run	<role>	Runs the binary for the role of the given name.
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
    if len(sys.argv) < 3:
	print 'Usage: b run <role>\nPlease provide the name of the role to run.'
    else:
        if not os.path.isfile('build/roles/${sys.argv[2]}'):
            call("./${sys.argv[2]}", cwd='build/roles')

# print 'Number of arguments:', len(sys.argv), 'arguments.'
# print 'Argument List:', str(sys.argv)

# call(['echo', 'hello'])

# call(['ls'])
# call(['pwd'])
# call('ls', cwd = './puppet')

# print 'hello'
