#!/usr/bin/python

import sys
import os
import shutil
from subprocess import call
from subprocess import Popen
from subprocess import PIPE

def print_command_summary():
    print """
Usage: b <command> [arguments]

NUbots Helper
----------------------------------------------------------------------
This script is an optional helper script for performing common tasks
related to building and running NUClearPort and related projects.

Command summary:
  - help              Show this help.
  - clean             Deletes the build directory.
  - cmake             Runs cmake in the build directory (creating it if it 
                      doesn't exist).
  - make [arg]...     Runs cmake, then make in the build directory (creating it
                      if it doesn't exist), passing any arguments to the make
                      command.
  - makej             Same as make, but runs 'make -j'.
  - run <role>        Runs the binary for the role of the given name.
  - debug <role>      Runs the binary for the role of the given name under gdb.
  - create_box <provider> Builds the nubots Vagrant box, for the given
                          virtualisation provider (vmware, or vitrualbox),
                          using Packer.
                          (the box is first deleted if it already exists)
"""

def clean():
    if os.path.exists('build'):
        shutil.rmtree('build')

def cmake():
    if not os.path.exists('build'):
        os.mkdir('build') 
    call(['cmake', '..'], cwd='build')


def make(args):
    call(['make'] + args, cwd='build')

def role_exists(role):
    return os.path.isfile("build/roles/{}".format(role))

def run(role):
    if role == '':
        print '''
Usage: b run <role>

Please provide the name of the role to run.
'''
    elif role_exists(role):
        call("./roles/{}".format(role), cwd='build/')
    else:
        print "The role '{}' does not exist or did not build correctly.".format(role)

def debug(role):
    if role == '':
        print '''
Usage: b debug <role>

Please provide the name of the role to debug.
'''
    elif role_exists(role):
        call(["gdb", "./roles/{}".format(role), "-ex", "r"], cwd='build/')
    else:
        print "The role '{}' does not exist or did not build correctly.".format(role)

def box_exists(box_name, provider):
    p1 = Popen(['vagrant', 'box', 'list'], stdout=PIPE)
    p2 = Popen(['grep', "'{0}.*({1}'".format(box_name, provider)], stdin=p1.stdout, stdout=PIPE)
    return p2.communicate()[0] != ''

def box_generated(provider):
    return os.path.isfile("packer/nubots-ubuntu-12-04-x86-{}.box".format(provider))

def packer_is_installed():
    return not call(['which', 'packer'])

def packer(provider):
    box_name = 'nubots-14.02'

    if box_generated(provider):
        call(['rm', 'nubots-ubuntu-12-04-x86-{}.box'.format(provider)], cwd='packer')
    
    call(['packer', 'build', '-only={}-iso'.format(provider), 'template.json'], cwd='packer')
    
    if box_generated(provider):
        if box_exists(box_name, provider):
            call(['vagrant', 'box', 'remove', box_name]) # TODO: only remove box for given provider
        
        call(['vagrant', 'box', 'add', box_name,
              'nubots-ubuntu-12-04-x86-{}.box'.format(provider)],
             cwd='packer')

def create_box(provider):
    if not packer_is_installed():
        print '''
The program packer must be installed to generate a box.

Please visit http://www.packer.io/ for information on installing packer.
'''
    elif provider == '' or (provider != 'vmware' and provider != 'virtualbox'):
        print '''
Usage: b create_box <provider>

Please provide the name of the provider for which to create the Vagrant box.
Allowable providers are:
    - vmware
    - virtualbox

The selected provider must be installed on the system to create a box.
'''
    else:
        packer(provider)

def unknown_command(command):
    print """
Unknown command: {}
Usage: b <command> [arguments]

Run './b help' for a command summary.
""".format(command)

def execute_command(command, args):
    arg0 = ''
    if len(args) >= 1:
        arg0 = args[0]

    if (command == '' or
       command == 'help' or
       command == '--help'):
        print_command_summary()
    elif command == 'clean':
        clean()
    elif command == 'cmake':
        cmake()
    elif command == 'makej':
        make(['-j2'])
    elif command == 'make':
        cmake()
        make(arguments)
    elif command == 'run':
        run(arg0)
    elif command == 'debug':
        debug(arg0)
    elif command == 'create_box':
        create_box(arg0)
    else:
        unknown_command(command)


# Prepare the command line arguments and perform the user's command:
command = ''
arguments = []

if len(sys.argv) >= 2:
    command = sys.argv[1]
if len(sys.argv) >= 3:
    arguments = sys.argv[2:]

try:
    execute_command(command, arguments)
except KeyboardInterrupt, e:
  print "\nThe process was interrupted by the Keyboard."
