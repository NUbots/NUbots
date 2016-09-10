#!/usr/bin/python
import sys
import generator.File
from google.protobuf.descriptor_pb2 import FileDescriptorSet

base_file = sys.argv[1]

with open('{}.pb'.format(base_file), 'r') as f:
    # Load the descriptor protobuf file
    d = FileDescriptorSet()
    d.ParseFromString(f.read())

    # Check that there is only one file
    assert(len(d.file) == 1)

    # Load the file
    b = generator.File.File(d.file[0])

    # Generate the c++ file
    header, impl = b.generate_cpp()

    with open('{}.cpp'.format(base_file), 'w') as f:
        f.write(impl)

    with open('{}.h'.format(base_file), 'w') as f:
        f.write(header)
