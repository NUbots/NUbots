#!/usr/bin/env python

import os
import b

from subprocess import call

def register(command):

    # Install help
    command.help = 'Format all the code in the codebase using clang-format'

def run(**kwargs):

    # Get our relevant directories
    for d in [os.path.join(b.project_dir, p) for p in ['module', 'shared', 'tests']]:
        for dirpath, dnames, fnames in os.walk(d):
            for f in fnames:
                if f.endswith('.h') or f.endswith('.c') or f.endswith('.cc') or f.endswith('.cxx') or f.endswith('.cpp') or f.endswith('.hpp') or f.endswith('.ipp'):
                    print('Formatting', f)
                    call(['clang-format-4.0', '-i', '-style=file', os.path.join(dirpath, f)])
