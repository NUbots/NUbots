#!/usr/bin/env python3

import textwrap


def indent(str, len=4):
    """Indent every line of the string by the value provided in length"""
    return '\n'.join([(' ' * len) + l for l in str.splitlines()])


def dedent(str):
    """Remove all common whitespace from each line of the string"""
    return textwrap.dedent(str)
