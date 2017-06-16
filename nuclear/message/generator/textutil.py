#!/usr/bin/env python3

import textwrap


def indent(str, len=4):
    return '\n'.join([(' ' * len) + l for l in str.splitlines()])


def dedent(str):
    return textwrap.dedent(str)
