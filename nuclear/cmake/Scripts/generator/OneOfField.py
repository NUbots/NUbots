#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2018 NUbots
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

import stringcase
from generator.Field import Field


class OneOfField:
    def __init__(self, oneof_name, oneof_fields, context):
        self.name = oneof_name

        # Some booleans to describe the type
        self.one_of = True
        self.type = "OneOf{}".format(stringcase.pascalcase(self.name))
        self.fqn = "{}.{}".format(context.fqn, self.type)
        self.default_value = "{}()".format(self.type)
        self.trivially_copyable = False

        self.map_type = False
        self.repeated = False
        self.pointer = False
        self.array_size = 0
        self.bytes_type = False
        self.basic = False

        self.oneof_fields = [Field(f, context) for f in oneof_fields]

        # Since our cpp_type is used a lot, precalculate it
        self.cpp_type = self.fqn.replace(".", "::")
        self.special_cpp_type = False

    def generate_cpp_header(self):
        return "{} {};".format(self.cpp_type, self.name)
