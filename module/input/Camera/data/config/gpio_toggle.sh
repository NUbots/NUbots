#!/bin/bash
echo 0 > /sys/class/gpio/gpio8/value
sleep 3
echo 1 > /sys/class/gpio/gpio8/value

