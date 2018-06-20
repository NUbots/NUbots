#!/usr/bin/env python3

import stringcase

from generator.Field import Field


class OneOfField:

    def __init__(self, oneof_name, oneof_fields, context):
        self.name = oneof_name

        # Some booleans to describe the type
        self.one_of = True
        self.type = 'OneOf{}'.format(stringcase.pascalcase(self.name))
        self.fqn = '{}.{}'.format(context.fqn, self.type)
        self.default_value = '{}()'.format(self.type)

        self.map_type = False
        self.repeated = False
        self.pointer = False
        self.array_size = 0
        self.bytes_type = False
        self.basic = False

        self.oneof_fields = [Field(f, context) for f in oneof_fields]

        # Since our cpp_type is used a lot, precalculate it
        self.cpp_type = self.fqn.replace('.', '::')
        self.special_cpp_type = False

    def generate_cpp_header(self):
        return '{} {};'.format(self.cpp_type, self.name)
