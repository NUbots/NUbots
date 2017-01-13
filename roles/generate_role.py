#!/usr/bin/env python3

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
import textwrap
import itertools

role_name = sys.argv[1]
banner_file = sys.argv[2]
module_path = sys.argv[3]
role_modules = sys.argv[4:]

# Open our output role file
with open(role_name, 'w') as file:

    # We use our NUClear header
    file.write('#include <nuclear>\n\n')

    # Add our module headers
    for module in role_modules:
        # Each module is given to us as Namespace::Namespace::Name.
        # we need to replace the ::'s with /'s so we can include them.

        # module::a::b::C
        # module/a/b/C/src/C.h

        # replace :: with /
        header = re.sub(r'::', r'/', module)
        # replace last name with src/name.h
        header = re.sub(r'([^\/]+)$', r'\1/src/\1.h', header)
        file.write('#include "{}"\n'.format(header))

    # Add our main function and include headers
    main = textwrap.dedent("""
        #include "utility/strutil/ansi.h"
        #include "utility/strutil/banner.h"

        using utility::strutil::Colour;

        int main(int argc, char** argv) {
        """)
    file.write(main)

    # Read our banner file
    with open(banner_file, 'r') as banner_file:
        banner = banner_file.read().split('\n')

    # Add our banner
    for banner_line in banner:

        # Add the initial cerr
        file.write('    std::cerr')

        # Split our line into sections for colours
        sections = [[k,len(list(g))] for k, g in itertools.groupby(banner_line)]

        for section in sections:

            # If it does not have colour print
            if section[0] in '# ':
                file.write(' << "' + (section[0] * section[1]) + '"')
            # Otherwise print the colour and then the hashes
            else:
                if section[0] in 'k':     # black
                    file.write(' << Colour::black')
                elif section[0] in 'K':   # darkgrey
                    file.write(' << Colour::brightblack')
                elif section[0] in 'r':   # darkred
                    file.write(' << Colour::red')
                elif section[0] in 'R':   # red
                    file.write(' << Colour::brightred')
                elif section[0] in 'g':   # darkgreen
                    file.write(' << Colour::green')
                elif section[0] in 'G':   # green
                    file.write(' << Colour::brightgreen')
                elif section[0] in 'y':   # darkyellow
                    file.write(' << Colour::yellow')
                elif section[0] in 'Y':   # yellow
                    file.write(' << Colour::brightyellow')
                elif section[0] in 'b':   # darkblue
                    file.write(' << Colour::blue')
                elif section[0] in 'B':   # blue
                    file.write(' << Colour::brightblue')
                elif section[0] in 'm':   # darkmagenta
                    file.write(' << Colour::magenta')
                elif section[0] in 'M':   # magenta
                    file.write(' << Colour::brightmagenta')
                elif section[0] in 'c':   # darkcyan
                    file.write(' << Colour::cyan')
                elif section[0] in 'C':   # cyan
                    file.write(' << Colour::brightcyan')
                elif section[0] in 'w':   # lightgrey
                    file.write(' << Colour::gray')
                elif section[0] in 'W':   # white
                    file.write(' << Colour::brightgray')
                else:
                    print("The banner file contains an invalid character", section)
                    exit(1)

                # Write the actual banner text
                file.write(' << "' + ('#' * section[1]) + '"')

        file.write(' << std::endl;\n');

    # Insert banner for the name of the executing role
    rolebanner = '    std::cerr << utility::strutil::banner("{0}");\n    std::cerr << std::endl;\n'.format(os.path.splitext(os.path.basename(role_name))[0])
    file.write(rolebanner)

    start = """

    NUClear::PowerPlant::Configuration config;
    unsigned int nThreads = std::thread::hardware_concurrency() + 2;
    config.threadCount = nThreads >= 4 ? nThreads : 4;

    NUClear::PowerPlant plant(config, argc, const_cast<const char**>(argv));
"""

    file.write(start)

    for module in role_modules:
        file.write('    std::cerr << "Installing " << "{0}" << std::endl;\n'.format(module))
        file.write('    plant.install<module::{0}>();\n'.format(module))

    end = """
    plant.start();
    return 0;
}
"""
    file.write(end)
