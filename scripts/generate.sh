#!/bin/bash
#
# File:   generate.sh
# Authors: 
#   Jake Woods <jake.f.woods@gmail.com>
#   Trent Houliston <trent@houliston.me>
#

# Ensure we have specified a name
if [ -n "$1" ] ; then
    role_name=$1;
else
    echo "You must specify a name";
    exit 1;
fi

# Ensure we've got at least one module
if [ -n "$2" ] ; then
    role_modules=$2;
else
    echo "You must specify at least one module";
    exit 1;
fi

# Delete the old role if it exists. We pipe to
# /dev/null to supress a message if it doesn't
# exist.
rm "$role_name" 2> /dev/null;

# Build up our headers.
# We always need NUClear.h
echo "#include <nuclear>" >> "$role_name";

# Add our module headers
for module in ${@:2}; do
    # Each module is given to us as Namespace::Namespace::Name.
    # we need to replace the ::'s with /'s so we can include them.
    echo "#include \"`echo "${module}" | sed -e \"s/::/\//g\"`.h\"" >> "$role_name";
done

# Add our main function.
cat >> "$role_name" << EOF

int main(int argc, char** argv) {
    NUClear::PowerPlant::Configuration config;
    config.threadCount = 4;

    NUClear::PowerPlant plant(config, argc, const_cast<const char**>(argv));
EOF

# Add our installs
for module in ${@:2}; do
    echo -e "\tplant.install<modules::${module}>();" >> "$role_name";
done

cat >> "$role_name" << EOF

    plant.start();
    return 0;
}
EOF
