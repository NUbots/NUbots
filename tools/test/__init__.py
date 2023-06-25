#!/usr/bin/env python3

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
