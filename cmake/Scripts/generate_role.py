#!/usr/bin/python
#
# File:   generate.py
# Authors:
#   Brendan Annable <brendan.annable@uon.edu.au>
#   Jake Woods <jake.f.woods@gmail.com>
#   Trent Houliston <trent@houliston.me>
#
import sys
import re
import os

# Ensure we have specified a name
if sys.argv[1]:
    role_name = sys.argv[1]
else:
    print 'You must specify a name'
    sys.exit(1)

# Ensure we've got at least one module
if sys.argv[2]:
    role_modules = sys.argv[2]
else:
    print 'You must specify at least one module'
    sys.exit(1)


with open(role_name, 'w') as file:
    # Build up our headers.
    # We always need NUClear.h
    file.write('#include <nuclear>\n\n')

    # Add our module headers
    for module in sys.argv[2:]:
        # Each module is given to us as Namespace::Namespace::Name.
        # we need to replace the ::'s with /'s so we can include them.

        # module::a::b::C
        # module/a/b/C/src/C.h

        # replace :: with /
        header = re.sub(r'::', r'/', module)
        # replace last name with src/name.h
        header = re.sub(r'([^\/]+)$', r'\1/src/\1.h', header)
        file.write('#include "modules/{0}"\n'.format(header))

    # Add our main function.
    main = """
#include "utility/strutil/ansi.h"
#include "utility/strutil/banner.h"

int main(int argc, char** argv) {

    auto white = utility::strutil::ANSISGR<
                   utility::strutil::ANSICode::BRIGHT,
                   utility::strutil::ANSICode::GRAY,
                   utility::strutil::ANSICode::BLACK_BACKGROUND>();
    auto yellow = utility::strutil::ANSISGR<
                    utility::strutil::ANSICode::BRIGHT,
                    utility::strutil::ANSICode::YELLOW,
                    utility::strutil::ANSICode::BLACK_BACKGROUND>();

    std::cout << std::endl << std::endl;
    std::cout << white << "######                    ######  ######           ######  " << yellow << "####################" << std::endl;
    std::cout << white << "#######                   ######  ######           ######  " << yellow << "####################" << std::endl;
    std::cout << white << "########                  ######  ######           ######  " << yellow << "###     ####     ###" << std::endl;
    std::cout << white << "#########                 ######  ######           ######  " << yellow << "###     ####     ###" << std::endl;
    std::cout << white << "##########                ######  ######           ######  " << yellow << " ########  ######## " << std::endl;
    std::cout << white << "###########               ######  ######           ######  " << yellow << "  ######    ######  " << std::endl;
    std::cout << white << "############              ######  ######           ######  " << yellow << "                    " << std::endl;
    std::cout << white << "###### ######             ######  ######           ######  " << yellow << "  ################  " << std::endl;
    std::cout << white << "######  ######            ######  ######           ######  " << yellow << " ################## " << std::endl;
    std::cout << white << "######   ######           ######  ######           ######  " << yellow << "###              ###" << std::endl;
    std::cout << white << "######    ######          ######  ######           ######  " << yellow << "###              ###" << std::endl;
    std::cout << white << "######     ######         ######  ######           ######  " << yellow << " ################## " << std::endl;
    std::cout << white << "######      ######        ######  ######           ######  " << yellow << "  ################  " << std::endl;
    std::cout << white << "######       ######       ######  ######           ######  " << yellow << "                    " << std::endl;
    std::cout << white << "######        ######      ######  ######           ######  " << yellow << "                 ###" << std::endl;
    std::cout << white << "######         ######     ######  ######           ######  " << yellow << "                 ###" << std::endl;
    std::cout << white << "######          ######    ######  ######           ######  " << yellow << "####################" << std::endl;
    std::cout << white << "######           ######   ######  ######           ######  " << yellow << "####################" << std::endl;
    std::cout << white << "######            ######  ######  ######           ######  " << yellow << "                 ###" << std::endl;
    std::cout << white << "######             ###### ######  ######           ######  " << yellow << "                 ###" << std::endl;
    std::cout << white << "######              ############  ######           ######  " << yellow << "                    " << std::endl;
    std::cout << white << "######               ###########  ######           ######  " << yellow << "  ###      #######  " << std::endl;
    std::cout << white << "######                ##########  ######           ######  " << yellow << " ###      ######### " << std::endl;
    std::cout << white << "######                 #########  ######           ######  " << yellow << "###      ###     ###" << std::endl;
    std::cout << white << "######                  ########   #####################   " << yellow << "###     ###      ###" << std::endl;
    std::cout << white << "######                   #######     #################     " << yellow << " #########      ### " << std::endl;
    std::cout << white << "######                    ######       #############       " << yellow << "  #######      ###  " << std::endl;
    std::cout << std::endl;

"""
    file.write(main)



    rolebanner = '    std::cout << utility::strutil::banner("{0}");\n    std::cout << std::endl;\n'.format(os.path.splitext(os.path.basename(role_name))[0])
    file.write(rolebanner)

    start = """

    NUClear::PowerPlant::Configuration config;
    config.threadCount = 8;

    NUClear::PowerPlant plant(config, argc, const_cast<const char**>(argv));
"""

    file.write(start)

    for module in sys.argv[2:]:
        file.write('    std::cout << "Installing " << "{0}" << std::endl;\n'.format(module))
        file.write('    plant.install<modules::{0}>();\n'.format(module))

    end = """
    plant.start();
    return 0;
}
"""
    file.write(end)
