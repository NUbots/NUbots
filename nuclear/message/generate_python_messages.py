#!/usr/bin/env python3
import sys
import os
import pkgutil
import shutil
import google.protobuf.message
from generator.textutil import dedent, indent


def recurse(messages, root, key='message', indent=0):
    keys = list(messages[key].keys())
    if 'path' in messages[key]:
        path = os.path.join(root, messages[key]['path'])
        header = '{}.h'.format(os.path.join(messages[key]['path'], key))

        if not os.path.isdir(path):
            os.makedirs(path, exist_ok=True)

        with open(os.path.join(path, '__init__.py'), 'a') as f:
            f.write('{}class {}:\n'.format(' ' * indent, key))
            f.write('{}    def include_path(): return "{}"\n'.format(' ' * indent, header))

        keys.remove('path')
        indent += 4

    for k in keys:
        recurse(messages[key], root, k, indent)


# Open up our message output directory to get our protobuf types
sys.path.append(sys.argv[1])
# Load all of the protobuf files as modules to use
for dir_name, subdir, files in os.walk(sys.argv[1]):
    modules = pkgutil.iter_modules(path=[dir_name])
    for loader, module_name, ispkg in modules:
        if module_name.endswith('pb2'):
            # Load our protobuf module
            module = loader.find_module(module_name).load_module(module_name)

# Now that we've imported them all get all the subclasses of protobuf message
messages = []

for message in google.protobuf.message.Message.__subclasses__():
    # Work out our original protobuf type
    pb_type = message.DESCRIPTOR.full_name.split('.')[1:]
    # Ignore messages that aren't ours
    if len(pb_type) > 0 and not pb_type[0] == 'protobuf':
        messages.append((message.DESCRIPTOR.full_name, message.DESCRIPTOR.file.name))

if os.path.isdir(os.path.join(sys.argv[2], 'message')):
    shutil.rmtree(os.path.join(sys.argv[2], 'message'))

files = {}
for message in messages:
    current = files
    fqn = message[0].replace('protobuf.', '').split('.')

    for element in fqn:
        if element not in current:
            current[element] = {}
        current = current[element]
    current['path'] = os.path.dirname(message[1])

recurse(files, sys.argv[2])
