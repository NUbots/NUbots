#!/bin/bash
# 
# File:   install.sh
# Author: Trent Houliston <trent@houliston.me>
#

if [ -z "$1" ] ; then
    echo "You must set the robots ip (robot=ip)";
    exit 1;
else
    robotIP=$1;
fi

if [ -n "$2" ] ; then
    config=$2
else
    config=""
fi

# Copy our robocup binary over
scp robocup "darwin@$robotIP:/home/darwin/"

# Overwrite configuration files
if [ "$config" == "update" ] || [ "$config" == "u" ] ;
then
    echo "Updating configuration files"
    rsync -avz -e ssh config "darwin@$robotIP:/home/darwin/"

# Update configuration files
elif [ "$config" == "new" ] || [ "$config" == "n" ] || [ "$config" == "" ] ;
then
    echo "Adding new configuration files only"
    rsync -avz --ignore-existing -e ssh config "darwin@$robotIP:/home/darwin/"

# Ignore configuration files
elif [ "$config" == "ignore" ] || [ "$config" == "i" ] ;
then
    echo "Ignoring configuration changes"

# Bad configuration parameters
else
    echo "The configuration parameters must be either one of update, new or ignore (u, n or i)";
    exit 1;
fi