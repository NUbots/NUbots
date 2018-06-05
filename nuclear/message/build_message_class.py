#!/usr/bin/env python3
import sys
import generator.File
from google.protobuf.descriptor_pb2 import FileDescriptorSet

base_file = sys.argv[1]

with open('{}.pb'.format(base_file), 'rb') as f:
    # Load the descriptor protobuf file
    d = FileDescriptorSet()
    d.ParseFromString(f.read())

    # Check that there is only one file
    assert(len(d.file) == 1)

    # Load the file
    b = generator.File.File(d.file[0], base_file)

    # Generate the c++ file
    header, impl, python = b.generate_cpp()

    with open('{}.h'.format(base_file), 'w') as f:
        f.write(header)

    with open('{}.cpp'.format(base_file), 'w') as f:
        f.write(impl)

    with open('{}.py.cpp'.format(base_file), 'w') as f:
        f.write(python)
