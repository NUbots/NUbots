#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2023 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

import unittest

from . import utility


def register(command):
    command.description = "Run all the python unit tests"

    command.add_argument("-v", "--verbose", dest="verbose", help="Verbose output", action="store_true", default=False)
    command.add_argument(
        "-f", "--failfast", dest="failfast", help="Stop the test run on first error", action="store_true", default=False
    )
    command.add_argument(
        "-b", "--buffer", dest="buffer", help="Don't output successes", action="store_true", default=False
    )


def run(verbose, failfast, buffer, **kwargs):
    verbosity = 1
    if verbose:
        verbosity = 2

    test_loader = unittest.defaultTestLoader

    # Load the tests we are going to run
    # See https://docs.python.org/3/library/unittest.html#unittest.TestLoader for ways to load modules
    utility_suite = test_loader.loadTestsFromModule(utility)

    test_runner = unittest.TextTestRunner(verbosity=verbosity, failfast=failfast, buffer=buffer)

    # Run each test
    test_runner.run(utility_suite)
