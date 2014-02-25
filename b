#!/usr/bin/python

import sys
import os
import shutil
from subprocess import call

numArgs = len(sys.argv)
command = ''
arg1 = ''
arg2 = ''

# print 'Number of arguments:', numArgs, 'arguments.'
# print 'Argument List:', str(sys.argv)

if numArgs >= 2:
  command = sys.argv[1]
  if numArgs >= 3:
    arg1 = sys.argv[2]

if numArgs == 1 or (numArgs > 1 and (command == 'help' or command == '--help')):
    # Print command summary
    print """
Usage: b <command> [arguments]

NUbots Helper
----------------------------------------------------------------------
This script is an optional helper script for performing common tasks
related to building and running NUClearPort and related projects.

Command summary:
  - help        Show this help.
  - clean       Deletes the build directory.
  - cmake       Runs cmake in the build directory (creating it if it doesn't exist).
  - make        Runs make in the build directory (creates it and runs cmake if it doesn't exist).
  - makej       Same as make, but runs 'make -j'.
  - run <role>  Runs the binary for the role of the given name.
  - debug <role> Runs the binary for the role of the given name under gdb.
  - create_box	Builds the nubots Vagrant box using Packer.
                (the box is first deleted if it already exists)
"""

elif command == 'clean':
    if os.path.exists('build'):
        shutil.rmtree('build')

elif command == 'cmake':
    if not os.path.exists('build'):
        os.mkdir('build') 
    call(['cmake', '..'], cwd='build')

elif command == 'make' or command == 'makej':
    if not os.path.exists('build'):
        os.mkdir('build') 
        call(['cmake', '..'], cwd='build')

    if command == 'make':
        call('make', cwd='build')
    else:
        call(['make', '-j2'], cwd='build')

elif command == 'run' or command == 'debug':
    if numArgs < 3:
	print '''
Usage: b run <role>

Please provide the name of the role to run.
'''
    elif os.path.isfile("build/roles/{}".format(arg1)):
        try:
          if command == 'run':
            call("./roles/{}".format(arg1), cwd='build/')
          else:
            call(["gdb", "./roles/{}".format(arg1), "-ex", "r"], cwd='build/')

        except KeyboardInterrupt, e:
          print "\nThe process was interrupted by the Keyboard."
    else:
        print "The role '{}' does not exist or did not build correctly.".format(arg1)

elif command == 'create_box':
  if os.path.isfile("packer/nubots-ubuntu-12-04-x86-virtualbox.box"):
    call(['rm', 'nubots-ubuntu-12-04-x86-virtualbox.box'], cwd='packer')
  if os.path.isfile("packer/nubots-ubuntu-12-04-x86-vmware.box"):
    call(['rm', 'nubots-ubuntu-12-04-x86-vmware.box'], cwd='packer')
  call(['packer', 'build', '-only=vmware-iso', 'template.json'], cwd='packer')
  call(['vagrant', 'box', 'remove', 'nubots-14.02'])
  if os.path.isfile("packer/nubots-ubuntu-12-04-x86-virtualbox.box"):
    call(['vagrant', 'box', 'add', 'nubots-14.02', 'nubots-ubuntu-12-04-x86-virtualbox.box'], cwd='packer')
  else:
    call(['vagrant', 'box', 'add', 'nubots-14.02', 'nubots-ubuntu-12-04-x86-vmware.box'], cwd='packer')

else:
  # Unknown command: print usage and help
  print """
Unknown command: {}
Usage: b <command> [arguments]

Run './b help' for a command summary.
""".format(command)
