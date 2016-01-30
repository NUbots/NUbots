Signal Catcher
==============

## Description

This module catches and handles signals.

## Usage

When SIGINT (interrupt - Ctrl+C) is received, SignalCatcher attempts to shut
down the NUClear power plant and terminate the program. If another SIGINT is
received (e.g. because shutting down the power plant did not fully terminate
the program) it immediately exits the program with return code 1.

If SIGSEGV (segmentation fault) is received, SignalCatcher throws a
`message::SegmentationFault` exception.

