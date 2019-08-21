# NUClear Roles
The NUClear roles system is a build and messaging system for the NUClear framework.
It uses on CMake and Python to manage the construction of various executables made up of a selection of modules.
These executables are called roles.

CMake is used as the main system for generating the libraries and executables that are used for the final binary.
Note that it utilises globbing to find the sources that are used for modules.
So if you add or remove a file, make sure you rerun cmake so that it locates the new files.

## Setup
NUClear Roles is designed to exist as a part of another repository as either a git subtree or git submodule.
In general subtrees are preferred to submodules as they allow you to make your own local changes to NUClear Roles and still be able to merge in upstream changes.
To setup NUClearRoles as a subtree follow the following steps.

- First create your repository where the NUClear Roles based system will live.
- Once you have a repository to attach to run the following command from the root directory of the repository
```bash
git subtree add --prefix nuclear https://github.com/Fastcode/NUClearRoles.git master --squash
```
- This will pull in NUClear Roles into the nuclear subdirectory ready for use by your system.

Once you have added NUClearRoles to your codebase you must then configure a CMakeLists.txt file to use it.
The structure of this CMakeLists.txt file should be as follows

```cmake
CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(<Project Name Here>)

# Set variables for NUClear Roles here, e.g. to set the banner file to a custom banner
SET(NUCLEAR_ROLE_BANNER_FILE "${PROJECT_SOURCE_DIR}/banner.png" CACHE PATH "The path the banner to print at the start of each role execution" FORCE)

# Finally Include the NUClear roles system
ADD_SUBDIRECTORY(nuclear)
```

CMake code that influences the variables used by NUClear Roles should come before the `ADD_SUBDIRECTORY` line.
CMake code that depends on variables created by NUClear Roles should come after that line.

It is also recommended that you make a symlink from nuclear/b to ./b to make it easier to access the functionality of the b script.

### Dependencies
NUClear roles has several dependences that must be met before you are able to build the system.
These dependencies are:

- NUClear
- Python3 with the following packages
  - Pillow
  - stringcase
- Optional dependencies:
  - [pybind11](https://github.com/pybind/pybind11) for Python module support
  - [Google Protobuf 3](https://developers.google.com/protocol-buffers/) (both c++ and python libraries) for Neutron messaging

The Python dependencies can be installed using the provided requirements file:
```bash
pip3 install -r requirements.txt
```

### Banner
NUClear roles generates an ansi coded banner at the top of ever role it runs.
This banner file is created from an image file that is provided using the CMake cache variable `NUCLEAR_ROLE_BANNER_FILE`

As the banner file is converted into ansi coloured unicode text, there are limitations on how the final result can look.
To ensure that your banner looks good when rendered you should consider the following advice.
- Ensure that your image is 160px wide or less.

    Many terminals when created are 80 columns wide.
    The resolution of the created unicode text is half the resolution of the image.
    This means a 160px wide image will make 80 columns of text.

- Try to make your logo in an editor using a 0.5 pixel aspect ratio.

    The resolution of the image will be divided by four for the vertical axis.
    This is done as text blocks are (almost) twice as high as they are wide.

- Try to make smooth gradients.

    When selecting colours for the text, each character can have 2 colours which are aranged into any combination of the 4 quadrants.
    The unicode characters `█ ▄ ▐ ▞ ▟ ▚ ▌ ▙ ▀ ▜ ▛` are used to colour images so if the image has complex gradients the combination of these will look less clear.

If you do not set your own banner file, ![the default banner](roles/banner.png) will be used

## Directories
There are six main directories that are used within the NUClear Roles system.
- `module` for where NUClear reactors
- `message` for message types
- `extension` for any NUClear DSL extensions
- `utility` for utility code is stored that is shared amongst NUClear Reactors

- `roles` for the declaration of NUClear roles (executables made up of modules)
- `tools` for any command line extensions for the `b` script

### Module
The module directory is where all NUClear Reactors are stored.
This directory can be selected to be in a non default location by using the CMake cache variable `NUCLEAR_MODULE_DIR`.
If this variable is not set it defaults to `module`.

Within a the modules folder, a strict directory structure must be maintained so that the NUClear Roles system can find and build your modules.
All modules must have the most outer namespace `module`.
Take an example module `Camera` which exists in `namespace module::input`.
This module must be located at `${NUCLEAR_MODULE_DIR}/input/Camera`.
Within this module folder there are three directories that may hold code for the system.
- `${NUCLEAR_MODULE_DIR}/input/Camera/src` holds all of the source code for the module
  - This directory must contain a header file with the same name as the module followed by hpp, hh or h. E.g. Camera.h. This header must declare the NUClear::Reactor with the same name (Camera).
- `${NUCLEAR_MODULE_DIR}/input/Camera/data` holds any non source code files. These will be copied to the build directory when building the code.
- `${NUCLEAR_MODULE_DIR}/input/Camera/test` holds any unit test source code.

### Message
This directory can be selected to be in a non default location by using the CMake cache variable `NUCLEAR_MESSAGE_DIR`.
If this variable is not set it defaults to `shared/message`.
It is highly recommended that the message, utility and extension folders share a common parent folder.
By default this common parent folder will be the `shared` folder

If the message directory (`${NUCLEAR_MESSAGE_DIR}`) contains a CMakeLists.txt file it will use this file to build the messages for the system.
It is assumed this CMakeLists.txt file will build all of the library code into a linkable library if required, and provide c++ headers for inclusion.
It must set the cmake cache variables `NUCLEAR_MESSAGE_LIBRARIES` to the libraries that it creates and also set `NUCLEAR_MESSAGE_INCLUDE_DIRS` to the include path for the c++ headers.

If the message directory does not contain a CMakeLists.txt it will default to using the Neutron messaging system for messages.


#### Neutron Messaging System
While NUClear is able to use any c++ type as a message when emitting/triggering, it is advantageous to be able to serialise data in order to send data over a network or save to a file.
The Neutron messaging system is designed to fill the gap using a system based on [Google Protocol Buffers](https://developers.google.com/protocol-buffers/).
It uses protocol buffers for serialisation, however instead of using their c++ classes, it generates simplified structures to make it easier to use them within code.

```
TODO the transformation from proto file to message class
TODO smart enums
```

Using the Neutron messaging system enables support for Python modules.
The messages sent to and from python are not serialised but instead, Neutron generates bindings to the actual c++ classes so you can have low latency communicaiton between the two languages.
Note that the support for Python NUClear modules is still in alpha YMMV.

### Extension/Utility
These two directories are handled in a similarly to each other.
The extension directory can be selected to a non default location  using the CMake cache variable `NUCLEAR_EXTENSION_DIR`.
The utility directory can be selected to a non default location  using the CMake cache variable `NUCLEAR_UTILITY_DIR`.

If the directory contains a CMakeLists.txt file it will use this file to build the extensions/utilities for the system.
This CMakeLists.txt file must ensure it sets `NUCLEAR_EXTENSION_INCLUDE_DIRS` and `NUCLEAR_EXTENSION_LIBRARIES` for extensions, or `NUCLEAR_UTILITY_INCLUDE_DIRS` and `NUCLEAR_UTILITY_LIBRARIES` for utilities.

If the folders do not contain a CMakeLists.txt file the default behaviour will be to build all c/cc/cpp files within that directory recursively.

### Roles
The NUClear roles directory contains a series of files named `<executablename>.role` where `<executablename>` is the name of the final binary that will be created.
This directory can be selected to be in a non default location by using the CMake cache variable `NUCLEAR_ROLES_DIR`.
If this variable is not set it defaults to `roles`.

The name of each module is described as a fully qualified c++ class without the initial module namespace.
These modules will then be installed in order when the executable is run.
This is important to note as it means modules that have dependencies on other modules may want to be lower in the list.
For example installing log handler modules should happen before installing other modules so their output can be seen.
It will use this name to locate the module so the directory structure must match the name.
An example of a role file would be:
```cmake
NUCLEAR_ROLE(
    # Some global modules that are useful
    extension::FileWatcher
    support::logging::ConsoleLogHandler

    # The Camera module we described earlier
    input::Camera
)

```
This role file would create an executable which had the modules `module::extension::FileWatcher`, `module::support::logging::ConsoleLogHandler` and `module::input::Camera`.
This file is a cmake file so you are able to use # to declare comments.

## Tools and the `./b` script
NUClear Roles comes with a small python tool called `b` that lives in nuclear/b.
This python tool is used to handle common functionality that you may wish to add to you system that takes advantage of information available in NUClear Roles.

By default the b script comes with only a single tool that can be accessed by ./b module generate <path_from_NUCLEAR_MODULE_DIR>
This tool generates a new NUClear module at `${NUCLEAR_MODULE_DIR}/<path>` which can save some of the effort of setting up the boilerplate code.

However the real power of the `./b` script is it's ability to be extended with new functionality.
The ./b script will look for a `tools` directory above the location where the actual b.py is located.
If it finds one, it will load all `.py` files in that directory and attempt to execute a register and run function on them.
This allows the tools to register new functionality to the b script that can be accessed from the command line.
For example three tools that exist in the NUbots codebase are:
- [this tool](https://github.com/NUbots/NUbots/blob/master/tools/install.py) installs NUClear roles systems onto remote systems using rsync
- [this tool](https://github.com/NUbots/NUbots/blob/master/tools/format.py) that formats all files using clang-format
- [this tool](https://github.com/NUbots/NUbots/blob/master/tools/decode.py) decodes `.nbs` files.

Any of the tools that are created in this way have access to the b import which provides access to several useful variables
- `b.nuclear_dir` the directory that NUClear Roles is stored in
- `b.project_dir` the directory above the NUClear Roles directory
- `b.cmake_cache` the variables stored in the cmake cache. It attempts to find this either in the `cwd` or in the `build`
- `b.binary_dir` the location of the project binary directory as reported by the cmake cache
- `b.source_dir` the location of the project source directory as reported by the cmake cache

## NUClear Modules
`TODO`

Variables
INCLUDES
LIBRARIES
SOURCES
DATA_FILES

## Other Useful Variables
Additional there are options that can be set in NUClear Roles using CMake Cache variables.
These options can influence how NUClear Roles generates systems.

- `NUCLEAR_ADDITIONAL_SHARED_LIBRARIES`
- `NUCLEAR_TEST_LIBRARIES`
- `NUCLEAR_ROLES_SYSTEM_DIR`

```
TODO descriptions
```

## NUClear Binary Stream (nbs) files
To make it easier to serialise streams of messages for storage sharing and playback, NUClear Roles defines a format for serialising messages to files.
This format is based on the Neutron messaging system and NUClear's networking protocol.
An `nbs` file has the following frames repeated continuously.

| Name      | Type    | Description                                                                       |
|-----------|---------|-----------------------------------------------------------------------------------|
| header    | char[3] | the header sequence 0xE2, 0x98, 0xA2 (the radioactive symbol ☢ in utf-8)          |
| size      | uint32  | the size of the frame after this byte in bytes                                    |
| timestamp | uint64  | the timestamp of this frame in microseconds. Does not have to be a utc timestamp. |
| hash      | uint64  | a 64 bit hash that identifies the type of the message                             |
| payload   | char*   | the serialised payload bytes                                                      |

All values within this format are little endian.
