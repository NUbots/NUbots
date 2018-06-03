#!/usr/bin/env python

import os
import b

from subprocess import call


def register(command):

    # Install help
    command.help = 'Format all the code in the codebase using clang-format'


def run(**kwargs):
    # Get our relevant directories
    for dirpath, dnames, fnames in os.walk(b.project_dir):
        for f in fnames:
            if f.endswith(('.h', '.c', '.cc', '.cxx', '.cpp', '.hpp', '.ipp', '.proto')):
                print('Formatting', f)
                call(['clang-format-6.0', '-i', '-style=file', os.path.join(dirpath, f)])

            if f.endswith(('.py')):
                print('Formatting', f)
                call(['yapf', '-i', os.path.join(dirpath, f)])
