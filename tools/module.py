#!/usr/bin/python

import sys
import os
import platform

class Module:
    def __init__(self, **kwargs):

        self.args = kwargs;

        # If we have a command to run then set it up to run
        if 'module_command' in kwargs:
            self.command = getattr(self, kwargs['module_command'])

    def generate(self):

        # Get our path
        path = self.args['path']

        # Check if the module already exists
        if os.path.exists('modules/{}'.format(path)):

            print("The path provided already exists.")
            print("Module generation aborted.")
            sys.exit(1)

        src_path =
        config_path =
        test_path =
        module_name =

    def run(self):
        self.command()

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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

""".format(module_name)

def surround_with_namespaces(code, namespaces):
    namespace_openings = ''
    namespace_closings = ''
    for namespace in namespaces:
        namespace_openings += 'namespace {} {{\n'.format(namespace)
        namespace_closings += '}\n'

    return '{0}\n{1}\n{2}'.format(namespace_openings, code, namespace_closings)

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

    return surround_with_include_guard("""#include <nuclear>

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


