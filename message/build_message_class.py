#!/usr/bin/python
import os
import sys
import generator.File
from google.protobuf.descriptor_pb2 import FileDescriptorSet, FieldOptions

# Add our cwd to the path so we can import generated python protobufs
# And extend our options with our MessageOptions
sys.path.append(os.getcwd())
from MessageOptions_pb2 import pointer, PointerType
FieldOptions.RegisterExtension(pointer)
PointerType = dict(PointerType.items())


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
