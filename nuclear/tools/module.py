#!/usr/bin/python

import os
import textwrap
import datetime

def register(command):

    # Module help
    command.help = 'Manage NUClear modules in the codebase'

    # Module subcommands
    subcommands = command.add_subparsers(dest='module_command')

    # Generate module subcommand
    generate_command = subcommands.add_parser('generate', help='Generate a new NUClear module based on a template')
    generate_command.add_argument('path', metavar='path', help='a path to the new module (from the module directory)')

def run(path='', **kwargs):
    if os.path.exists('module/{}'.format(path)):
        print("The path provided already exists.")
        print("Module generation aborted.")
    else:

        # Calculate all of our file paths
        path = 'module/{}'.format(path)
        src_path = '{}/src'.format(path)
        tests_path = '{}/tests'.format(path)
        config_path = '{}/config'.format(path)
        module_name = path.split('/')[-1]

        # Create the required directories
        os.makedirs(path)
        os.makedirs(src_path)
        os.makedirs(tests_path)
        os.makedirs(config_path)

        # Split our provided path
        parts = path.split('/')

        # Write all of our files
        with open('{}/CMakeLists.txt'.format(path), "w") as output:
            output.write(generate_cmake(parts))
        with open('{}/README.md'.format(path), "w") as output:
            output.write(generate_readme(parts))
        with open('{0}/{1}.h'.format(src_path, module_name), "w") as output:
            output.write(generate_header(parts))
        with open('{0}/{1}.cpp'.format(src_path, module_name), "w") as output:
            output.write(generate_cpp(parts))
        with open('{0}/{1}Test.cpp'.format(tests_path, module_name), "w") as output:
            output.write(generate_test(parts))

        with open('{0}/{1}.yaml'.format(config_path, module_name), 'a'):
            pass


def generate_cmake(parts):
    return textwrap.dedent("""\
        # Build our NUClear module
        NUCLEAR_MODULE()
        """)

def generate_header(parts):
    template = textwrap.dedent("""\
        /*
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
         * Copyright {year} NUbots <nubots@nubots.net>
         */

        #ifndef {define}
        #define {define}

        #include <nuclear>

        {openNamespace}

            class {className} : public NUClear::Reactor {{

            public:
                /// @brief Called by the powerplant to build and setup the {className} reactor.
                explicit {className}(std::unique_ptr<NUClear::Environment> environment);
            }};

        {closeNamespace}

        #endif  // {define}
        """)

    return template.format(year=datetime.datetime.now().year
                         , define='{}_H'.format('_'.join([p.upper() for p in parts]))
                         , className=parts[-1]
                         , openNamespace = '\n'.join(['namespace {} {{'.format(x) for x in parts[:-1]])
                         , closeNamespace = '\n'.join('}' * (len(parts) - 1)))

def generate_cpp(parts):
    template = textwrap.dedent("""\
        /*
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
         * Copyright {year} NUbots <nubots@nubots.net>
         */

        #include "{className}.h"

        #include "extension/Configuration.h"

        {openNamespace}

            using extension::Configuration;

            {className}::{className}(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)) {{

                on<Configuration>("{className}.yaml").then([this] (const Configuration& config) {{
                    // Use configuration here from file {className}.yaml
                }});
            }}
        {closeNamespace}
        """)

    return template.format(year=datetime.datetime.now().year
                         , className=parts[-1]
                         , openNamespace = '\n'.join(['namespace {} {{'.format(x) for x in parts[:-1]])
                         , closeNamespace = '\n'.join(['}' for x in parts[:-1]]))

def generate_readme(parts):
    template = textwrap.dedent("""\
        {className}
        {classNameTitle}

        ## Description


        ## Usage


        ## Emits


        ## Dependencies

        """)

    return template.format(className=parts[-1]
                         , classNameTitle = len(parts[-1]) * '='
                         , closeNamespace = '\n'.join(['}' for x in parts[:-1]]))
    pass

def generate_test(parts):
    template = textwrap.dedent("""\
        /*
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
         * Copyright {year} NUbots <nubots@nubots.net>
         */

        // Uncomment this line when other test files are added
        //#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
        //#include <catch.hpp>

        // Remove this line when test files are added
        int main() {{ return 0; }}
        """)

    return template.format(year=datetime.datetime.now().year)


