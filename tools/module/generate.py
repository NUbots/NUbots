#!/usr/bin/env python3

import os
import sys
import textwrap

import b


def register(command):

    # Module help
    command.description = "Generate a new NUClear Roles module at the provided location"

    # Module subcommands
    command.add_argument("path", metavar="path", help="a path to the new module (from the module directory)")
    command.add_argument("--director", action="store_true", help="generates a module as a Director behaviour module")


def run(path, director, **kwargs):
    # We use the default "module" directory for modules
    module_path = "module"

    # Calculate all of our file paths
    path = os.path.join(module_path, path)
    src_path = os.path.join(path, "src")
    config_path = os.path.join(path, "data", "config")
    module_name = os.path.split(path)[-1]

    # Check if the path already exists
    if os.path.exists(path):
        sys.stderr.write("The path provided already exists.\n")
        sys.stderr.write("Module generation aborted.\n")
        sys.exit(1)

    print("Module directory", module_path)
    print("Creating directories")

    # Create the required directories
    os.makedirs(path)
    print("\t", path)
    os.makedirs(src_path)
    print("\t", src_path)
    os.makedirs(config_path)
    print("\t", config_path)

    # Split our provided path
    parts = ["module"] + os.path.relpath(path, module_path).split(os.sep)

    print("Generating files")

    # Write all of our files
    with open(os.path.join(path, "CMakeLists.txt"), "w") as output:
        output.write(generate_cmake(parts))
        print("\t", os.path.join(path, "CMakeLists.txt"))

    with open(os.path.join(path, "README.md"), "w") as output:
        output.write(generate_readme(parts))
        print("\t", os.path.join(src_path, "README.md"))

    with open(os.path.join(src_path, "{}.hpp".format(module_name)), "w") as output:
        output.write(generate_header(parts, director))
        print("\t", os.path.join(src_path, "{}.hpp".format(module_name)))

    with open(os.path.join(src_path, "{}.cpp".format(module_name)), "w") as output:
        output.write(generate_cpp(parts, director))
        print("\t", os.path.join(src_path, "{}.cpp".format(module_name)))

    with open(os.path.join(config_path, "{}.yaml".format(module_name)), "a") as output:
        output.write("# Controls the minimum log level that NUClear log will display\n")
        output.write("log_level: INFO\n")
        print("\t", os.path.join(config_path, "{}.yaml".format(module_name)))


def generate_cmake(parts):
    return textwrap.dedent(
        """\
        # Build our NUClear module
        nuclear_module()
        """
    )


def generate_header(parts, director):
    director_template = textwrap.dedent(
        """\
        #ifndef {define}
        #define {define}

        #include <nuclear>

        #include "extension/Behaviour.hpp"

        namespace {namespace} {{

        class {className} : public ::extension::behaviour::BehaviourReactor {{
        private:
            /// @brief Stores configuration values
            struct Config {{
            }} cfg;

        public:
            /// @brief Called by the powerplant to build and setup the {className} reactor.
            explicit {className}(std::unique_ptr<NUClear::Environment> environment);
        }};

        }}  // namespace {namespace}

        #endif  // {define}
        """
    )

    normal_template = textwrap.dedent(
        """\
        #ifndef {define}
        #define {define}

        #include <nuclear>

        namespace {namespace} {{

        class {className} : public NUClear::Reactor {{
        private:
            /// @brief Stores configuration values
            struct Config {{
            }} cfg;

        public:
            /// @brief Called by the powerplant to build and setup the {className} reactor.
            explicit {className}(std::unique_ptr<NUClear::Environment> environment);
        }};

        }}  // namespace {namespace}

        #endif  // {define}
        """
    )

    if director:
        template = director_template
    else:
        template = normal_template

    return template.format(
        define="{}_HPP".format("_".join([p.upper() for p in parts])),
        className=parts[-1],
        namespace="::".join([x for x in parts[:-1]]),
    )


def generate_cpp(parts, director):
    director_template = textwrap.dedent(
        """\
        #include "{className}.hpp"

        #include "extension/Behaviour.hpp"
        #include "extension/Configuration.hpp"

        namespace {namespace} {{

        using extension::Configuration;

        {className}::{className}(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {{

            on<Configuration>("{className}.yaml").then([this](const Configuration& config) {{
                // Use configuration here from file {className}.yaml
                this->log_level = config["log_level"].as<NUClear::LogLevel>();
            }});
        }}

        }}  // namespace {namespace}
        """
    )
    normal_template = textwrap.dedent(
        """\
        #include "{className}.hpp"

        #include "extension/Configuration.hpp"

        namespace {namespace} {{

        using extension::Configuration;

        {className}::{className}(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {{

            on<Configuration>("{className}.yaml").then([this](const Configuration& config) {{
                // Use configuration here from file {className}.yaml
                this->log_level = config["log_level"].as<NUClear::LogLevel>();
            }});
        }}

        }}  // namespace {namespace}
        """
    )

    if director:
        template = director_template
    else:
        template = normal_template

    return template.format(className=parts[-1], namespace="::".join([x for x in parts[:-1]]))


def generate_readme(parts):
    template = textwrap.dedent(
        """\
        # {className}

        ## Description


        ## Usage


        ## Consumes


        ## Emits


        ## Dependencies

        """
    )

    return template.format(className=parts[-1])
