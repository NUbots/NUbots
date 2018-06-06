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
    generate_command.add_argument(
        'language',
        metavar='language',
        choices=['C++', 'c++', 'cpp', 'cxx', 'Python', 'python', 'py'],
        default='C++',
        help='Language to use for the module, C++ or Python [default=C++]'
    )


def run(path, **kwargs):
    # Try to get our actual module directory from the cmake cache
    if 'NUCLEAR_MODULE_DIR' in b.cmake_cache:
        module_path = os.path.join(b.source_dir, b.cmake_cache['NUCLEAR_MODULE_DIR'])
    else:
        sys.stderr.write('Warning: the system couldn\'t find the real module directory.')
        sys.stderr.write('defaulting to module\n')
        module_path = 'module'

    # Calculate all of our file paths
    path = os.path.join(module_path, path)
    src_path = os.path.join(path, 'src')
    tests_path = os.path.join(path, 'tests')
    config_path = os.path.join(path, 'data', 'config')
    module_name = os.path.split(path)[-1]

    # Check if the path already exists
    if os.path.exists(path):
        sys.stderr.write('The path provided already exists.\n')
        sys.stderr.write('Module generation aborted.\n')
        sys.exit(1)

    if language not in ['C++', 'c++', 'cpp', 'cxx', 'Python', 'python', 'py']:
        sys.stderr.write('The language provided is invalid.\n')
        sys.stderr.write('Module generation aborted.\n')
        sys.exit(1)

    module_language = 'CPP' if language in ['C++', 'c++', 'cpp', 'cxx'] else 'PYTHON'

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
    parts = ['module'] + os.path.relpath(path, module_path).split(os.sep)

    print('Generating files')

    # Write all of our files
    with open(os.path.join(path, 'CMakeLists.txt'), "w") as output:
        output.write(generate_cmake(parts))
        print('\t', os.path.join(path, 'CMakeLists.txt'))

    with open(os.path.join(path, 'README.md'), "w") as output:
        output.write(generate_readme(parts))
        print('\t', os.path.join(src_path, 'README.md'))

    if module_language is 'CPP':
        with open(os.path.join(src_path, '{}.h'.format(module_name)), "w") as output:
            output.write(generate_header(parts))
            print('\t', os.path.join(src_path, '{}.h'.format(module_name)))

        with open(os.path.join(src_path, '{}.cpp'.format(module_name)), "w") as output:
            output.write(generate_cpp(parts))
            print('\t', os.path.join(src_path, '{}.cpp'.format(module_name)))

    else:
        with open(os.path.join(src_path, '{}.py'.format(module_name)), "w") as output:
            output.write(generate_python(parts))
            print('\t', os.path.join(src_path, '{}.py'.format(module_name)))

    with open(os.path.join(tests_path, '{}.cpp'.format(module_name)), "w") as output:
        output.write(generate_test(parts))
        print('\t', os.path.join(tests_path, '{}.cpp'.format(module_name)))

    with open(os.path.join(config_path, '{}.yaml'.format(module_name)), 'a'):
        print('\t', os.path.join(config_path, '{}.yaml'.format(module_name)))


def generate_cmake(parts, language):
    return textwrap.dedent(
        """\
        # Build our NUClear module
        NUCLEAR_MODULE(LANGUAGE "{module_languge}")
        """
    ).format(module_languge=language)


def generate_header(parts):
    template = textwrap.dedent(
        """\
        #ifndef {define}
        #define {define}

        #include <nuclear>

        {open_namespace}

            class {class_name} : public NUClear::Reactor {{

            public:
                /// @brief Called by the powerplant to build and setup the {class_name} reactor.
                explicit {class_name}(std::unique_ptr<NUClear::Environment> environment);
            }};

        {close_namespace}

        #endif  // {define}
        """
    )

    return template.format(
        define='{}_H'.format('_'.join([p.upper() for p in parts])),
        class_name=parts[-1],
        open_namespace='\n'.join(['namespace {} {{'.format(x) for x in parts[:-1]]),
        close_namespace='\n'.join('}' * (len(parts) - 1))
    )


def generate_cpp(parts):
    template = textwrap.dedent(
        """\
        #include "{class_name}.h"

        #include "extension/Configuration.h"

        {open_namespace}

            using extension::Configuration;

            {class_name}::{class_name}(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)) {{

                on<Configuration>("{class_name}.yaml").then([this] (const Configuration& config) {{
                    // Use configuration here from file {class_name}.yaml
                }});
            }}
        {close_namespace}
        """
    )

    return template.format(
        class_name=parts[-1],
        open_namespace='\n'.join(['namespace {} {{'.format(x) for x in parts[:-1]]),
        close_namespace='\n'.join(['}' for x in parts[:-1]])
    )


def generate_readme(parts):
    template = textwrap.dedent(
        """\
        {class_name}
        {class_name_title}

        ## Description


        ## Usage


        ## Emits


        ## Dependencies

        """
    )

    return template.format(class_name=parts[-1], class_name_title=len(parts[-1]) * '=')


def generate_test(parts):
    template = textwrap.dedent(
        """\
        // Uncomment this line when other test files are added
        //#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
        //#include <catch.hpp>

        // Remove this line when test files are added
        int main() {{ return 0; }}
        """
    )

    return template.format()


def generate_python(parts):
    template = textwrap.dedent(
        """\
        #!/usr/bin/env python3

        from nuclear import Reactor, on, Trigger, Single, With, Every

        @Reactor
        class {class_name}(object):
            def __init__(self):
                # Constructor for {class_name}

            # Disabled until extension bindings and made
            # @on(Configuration('{class_name}.yaml'))
            # def {class_name}_configfuration(self, config):
            #     # Use configuration here from file {class_name}.yaml
        """
    )

    return template.format(class_name=parts[-1])
