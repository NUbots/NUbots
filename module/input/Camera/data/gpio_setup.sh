#!/bin/bash
echo 8 > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio8/direction
echo 1 > /sys/class/gpio/gpio8/value
chown nubots:nubots /sys/class/gpio/gpio8/value
