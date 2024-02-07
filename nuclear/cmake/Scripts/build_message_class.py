#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2016 NUbots
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
import sys

import generator.File
from google.protobuf.descriptor_pb2 import FileDescriptorSet

base_file = sys.argv[1]

with open("{}.pb".format(base_file), "rb") as f:
    # Load the descriptor protobuf file
    d = FileDescriptorSet()
    d.ParseFromString(f.read())

    # Check that there is only one file
    assert len(d.file) == 1

    # Load the file
    b = generator.File.File(d.file[0], base_file)

    # Generate the c++ file
    header, impl, python = b.generate_cpp()

    with open("{}.hpp".format(base_file), "w") as f:
        f.write(header)

    with open("{}.cpp".format(base_file), "w") as f:
        f.write(impl)

    with open("{}.py.cpp".format(base_file), "w") as f:
        f.write(python)
