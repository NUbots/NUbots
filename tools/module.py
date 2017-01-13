#!/usr/bin/env python

import b
import os
import sys
import textwrap

def register(command):

    # Module help
    command.help = 'Manage NUClear modules in the codebase'

    # Module subcommands
    subcommands = command.add_subparsers(dest='module_command')

    # Generate module subcommand
    generate_command = subcommands.add_parser('generate', help='Generate a new NUClear module based on a template')
    generate_command.add_argument('path', metavar='path', help='a path to the new module (from the module directory)')

def run(path, **kwargs):
    # Try to get our actual module directory from the cmake cache
    if 'NUCLEAR_MODULE_DIR' in b.cmake_cache:
        module_path = b.cmake_cache['NUCLEAR_MODULE_DIR']
    else:
        sys.stderr.write('Warning: the system couldn\'t find the real module directory.')
        sys.stderr.write('defaulting to module\n')
        module_path = 'module'

    # Calculate all of our file paths
    path = '{}/{}'.format(module_path, path)
    src_path = '{}/src'.format(path)
    tests_path = '{}/tests'.format(path)
    config_path = '{}/data/config'.format(path)
    module_name = path.split('/')[-1]

    # Check if the path already exists
    if os.path.exists(path):
        sys.stderr.write('The path provided already exists.\n')
        sys.stderr.write('Module generation aborted.\n')
        sys.exit(1)

    print('Module directory', module_path)
    print('Creating directories')

    # Create the required directories
    os.makedirs(path)
    print('\t', path)
    os.makedirs(src_path)
    print('\t', src_path)
    os.makedirs(tests_path)
    print('\t', tests_path)
    os.makedirs(config_path)
    print('\t', config_path)

    # Split our provided path
    parts = ['module'] + os.path.relpath(path, module_path).split('/')

    print('Generating files')

    # Write all of our files
    with open('{}/CMakeLists.txt'.format(path), "w") as output:
        output.write(generate_cmake(parts))
        print('\t', '{}/CMakeLists.txt'.format(path))

    with open('{}/README.md'.format(path), "w") as output:
        output.write(generate_readme(parts))
        print('\t', '{}/README.md'.format(path))

    with open('{0}/{1}.h'.format(src_path, module_name), "w") as output:
        output.write(generate_header(parts))
        print('\t', '{0}/{1}.h'.format(src_path, module_name))

    with open('{0}/{1}.cpp'.format(src_path, module_name), "w") as output:
        output.write(generate_cpp(parts))
        print('\t', '{0}/{1}.cpp'.format(src_path, module_name))

    with open('{0}/{1}Test.cpp'.format(tests_path, module_name), "w") as output:
        output.write(generate_test(parts))
        print('\t', '{0}/{1}Test.cpp'.format(tests_path, module_name))

    with open('{0}/{1}.yaml'.format(config_path, module_name), 'a'):
        print('\t', '{0}/{1}.yaml'.format(config_path, module_name))


def generate_cmake(parts):
    return textwrap.dedent("""\
        # Build our NUClear module
        NUCLEAR_MODULE()
        """)

def generate_header(parts):
    template = textwrap.dedent("""\
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

    return template.format(define='{}_H'.format('_'.join([p.upper() for p in parts]))
                         , className=parts[-1]
                         , openNamespace = '\n'.join(['namespace {} {{'.format(x) for x in parts[:-1]])
                         , closeNamespace = '\n'.join('}' * (len(parts) - 1)))

def generate_cpp(parts):
    template = textwrap.dedent("""\
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

    return template.format(className=parts[-1]
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

def generate_test(parts):
    template = textwrap.dedent("""\
        // Uncomment this line when other test files are added
        //#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
        //#include <catch.hpp>

        // Remove this line when test files are added
        int main() {{ return 0; }}
        """)

    return template.format()


