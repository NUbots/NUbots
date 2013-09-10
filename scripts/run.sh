#!/bin/bash
# 
# File:   run.sh
# Author: Trent Houliston <trent@houliston.me>
#

if [ -z "$1" ] ; then
    echo "You must set the robots ip (robot=ip)";
    exit 1;
else
    robotIP=$1;
fi

# Execute the robocup binary on the remote system
ssh "darwin@$robotIP" "/home/darwin/robocup"