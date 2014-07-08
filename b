#!/usr/bin/python

import sys
import os
import shutil
import platform
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
  - cmake_ninja       Runs cmake to generate a Ninja build.
  - make [arg]...     Runs cmake, then make in the build directory (creating it
                      if it doesn't exist), passing any arguments to the make
                      command.
  - makej             Same as make, but runs 'make -j'.
  - ninja [arg]...    Same as the make command, but runs ninja instead.
  - run <role>        Runs the binary for the role of the given name.
  - debug <role>      Runs the binary for the role of the given name under gdb.
  - create_box <provider> Builds the nubots Vagrant box, for the given
                          virtualisation provider (vmware, or vitrualbox),
                          using Packer.
                          (the box is first deleted if it already exists)
  - module            Allows management of NUClear modules
"""

def clean():
    if os.path.exists('build'):
        shutil.rmtree('build')

def cmake():
    if not os.path.exists('build'):
        os.mkdir('build')
    call(['cmake', '..'], cwd='build')

def make(args):
    if not os.path.exists('build'):
        cmake()
    call(['make'] + args, cwd='build')

def cmake_ninja():
    if not os.path.exists('build'):
        os.mkdir('build')
    call(['cmake', '..', '-G', 'Ninja'], cwd='build')

def ninja(args):
    if not os.path.exists('build'):
        cmake_ninja()
    call(['ninja'] + args, cwd='build')

def role_exists(role):
    return os.path.isfile("build/bin/{}".format(role))

def run(role):
    if role == '':
        print '''
Usage: b run <role>

Please provide the name of the role to run.
'''
    elif role_exists(role):
        env = os.environ.copy()
        # env['LD_LIBRARY_PATH'], "bin/lib:{}".format(env["LD_LIBRARY_PATH"]))
        env['LD_LIBRARY_PATH'] = "bin/lib"
        call(["./bin/{}".format(role)], cwd='build/', env=env)
    else:
        print "The role '{}' does not exist or did not build correctly.".format(role)

def debug(role):
    if role == '':
        print '''
Usage: b debug <role>

Please provide the name of the role to debug.
'''
    elif role_exists(role):
        env = os.environ.copy()
        # env['LD_LIBRARY_PATH'], "bin/lib:{}".format(env["LD_LIBRARY_PATH"]))
        env['LD_LIBRARY_PATH'] = "bin/lib"
        call(["gdb", "./bin/{}".format(role), "-ex", "r"], cwd='build/', env=env)
    else:
        print "The role '{}' does not exist or did not build correctly.".format(role)

def box_exists(box_name, provider):
    p1 = Popen(['vagrant', 'box', 'list'], stdout=PIPE)
    p2 = Popen(['grep', "'{0}.*({1}'".format(box_name, provider)], stdin=p1.stdout, stdout=PIPE)
    return p2.communicate()[0] != ''

def box_generated(provider):
    return os.path.isfile("packer/nubots-ubuntu-14-04-x86-{}.box".format(provider))

def packer_is_installed():
    cmd = "where" if platform.system() == "Windows" else "which" #which is where on windows
    return not call([cmd, 'packer'])

def packer(provider):
    box_name = 'nubots-14.04'

    if box_generated(provider):
        call(['rm', 'nubots-ubuntu-14-04-x86-{}.box'.format(provider)], cwd='packer')

    call(['packer', 'build', '-only={}-iso'.format(provider), 'template.yaml'], cwd='packer')

    if box_generated(provider):
        if box_exists(box_name, provider):
            call(['vagrant', 'box', 'remove', box_name]) # TODO: only remove box for given provider

        call(['vagrant', 'box', 'add', box_name,
              'nubots-ubuntu-14-04-x86-{}.box'.format(provider)],
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

def build_license_string(module_name):
    return """/*
 * This file is part of NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

""".format(module_name)

def surround_with_namespaces(code, namespaces):
    namespace_openings = ''
    namespace_closings = ''
    for namespace in namespaces:
        namespace_openings += 'namespace {} {{\n'.format(namespace)
        namespace_closings += '}\n'

    return """{0}
{1}
{2}""".format(namespace_openings, code, namespace_closings)

def surround_with_include_guard(code, path):
    guard = '_'.join(path.split('/')).upper()

    return """#ifndef {0}_H
#define {0}_H

{1}

#endif""".format(guard, code)

def build_module_header(path):
    module_name = path.split('/')[-1]
    module_namespaces = path.split('/')[:-1]

    module_class = """    class {0} : public NUClear::Reactor {{
    public:
        /// @brief Called by the powerplant to build and setup the {0} reactor.
        explicit {0}(std::unique_ptr<NUClear::Environment> environment);
    }};
""".format(module_name)

    code = surround_with_namespaces(module_class, module_namespaces)

    return surround_with_include_guard("""
#include <nuclear>
{}""".format(code), path)

def build_module_implementation(path):
    module_name = path.split('/')[-1]
    module_namespaces = path.split('/')[:-1]

    constructor = """    {0}::{0}(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {{

    }}
""".format(module_name)

    code = surround_with_namespaces(constructor, module_namespaces)

    return """#include "{0}.h"
#include <nuclear>

{1}
""".format(module_name, code)

def create_nuclear_module(path):
    src_path = '{}/src'.format(path)
    tests_path = '{}/tests'.format(path)
    config_path = '{}/config'.format(path)
    module_name = path.split('/')[-1]

    os.makedirs(path)
    os.makedirs(src_path)
    os.makedirs(tests_path)
    os.makedirs(config_path)

    with open('{}/CMakeLists.txt'.format(path), "w") as text_file:
        text_file.write('# Build our NUClear module\nNUCLEAR_MODULE()')

    with open('{}/README.md'.format(path), "w") as text_file:
        text_file.write('{0}\n{1}\n\n'.format(module_name, len(module_name) * '='))
        text_file.write('\n\n\n## '.join(['## Description', 'Usage', 'Emits', 'Dependencies']))

    with open('{0}/{1}.h'.format(src_path, module_name), "w") as text_file:
        text_file.write(build_license_string(module_name))
        text_file.write(build_module_header(path))

    with open('{0}/{1}.cpp'.format(src_path, module_name), "w") as text_file:
        text_file.write(build_license_string(module_name))
        text_file.write(build_module_implementation(path))

    with open('{0}/{1}Test.cpp'.format(tests_path, module_name), "w") as text_file:
        text_file.write(build_license_string(module_name))
        text_file.write("""\n
#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <catch.hpp>
""")


def module_generate(args):
    path = ''
    if len(args) >= 1:
        path = args[0]

    if path == '':
        print """
You need to provide the path to the module to generate.
Usage: b module generate <module-path>

e.g. The command:

    $ ./b module generate platform/darwin/HardwareIO

would generate a new module 'modules::platform::darwin::HardwareIO' in the
folder 'modules/platform/darwin/HardwareIO'.
"""
    elif os.path.exists('modules/{}'.format(path)):
        print """
The path provided already exists.
Module generation aborted.
"""
    else:
        create_nuclear_module('modules/{}'.format(path))



def module(command, args):
    arg0 = ''
    if len(args) >= 1:
        arg0 = args[0]

    if command == '':
        print """
Module requires a sub-command.
Usage: b module <sub-command> [arguments]

Available sub-commands are:
    - generate
"""
    elif command == 'generate':
        module_generate(args)
    else:
        print """
Unknown module sub-command: {}
Usage: b module <sub-command> [arguments]

Run './b help' for a command summary.
""".format(command)

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
    elif command == 'make':
        cmake()
        make(arguments)
    elif command == 'makej':
        cmake()
        make(['-j2'])

    elif command == 'cmake_ninja':
        cmake_ninja()
    elif command == 'ninja':
        cmake_ninja()
        ninja(arguments)

    elif command == 'run':
        run(arg0)
    elif command == 'debug':
        debug(arg0)

    elif command == 'create_box':
        create_box(arg0)
    elif command == 'module':
        module(arg0, args[1:])
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
