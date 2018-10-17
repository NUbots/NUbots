#!/usr/bin/env python3
import sys
import os
import pkgutil
import shutil
import google.protobuf.message


# Act as a file so we can still use a with statement
class Dummysink(object):

    def write(self, data):
        pass  # ignore the data

    def __enter__(self):
        return self

    def __exit__(*x):
        pass


def datasink(filename, mode):
    if filename:
        return open(filename, mode)
    else:
        return Dummysink()


def recurse(messages, root, log_file, key='message', indent=0):
    keys = list(messages[key].keys())
    if 'file' in messages[key]:
        header = messages[key]['file']
        path = os.path.dirname(os.path.join(root, header))
        py_file = os.path.join(path, '__init__.py')

        # Make sure the directory chain exists
        if not os.path.isdir(path):
            os.makedirs(path, exist_ok=True)

        # Make sure __init__.py exists for all folders in the chain
        base = root
        for component in header.split(os.sep)[:-1]:
            module_file = os.path.join(base, component, '__init__.py')
            if not os.path.exists(module_file):
                open(module_file, 'a').close()
                log_file.write('{}\n'.format(module_file))
            base = os.path.join(base, component)

        # Write new class to the file
        with open(py_file, 'a') as f:
            f.write('{}class {}:\n'.format(' ' * indent, key))
            f.write('{}    def include_path(): return "{}"\n'.format(' ' * indent, header))

        log_file.write('{}\n'.format(py_file))
        keys.remove('file')
        indent += 4

    for k in keys:
        recurse(messages[key], root, log_file, k, indent)


# sys.argv = [
#     script_name,
#     message_module_path,
#     output_directory,
#     log_file_path
# ]
message_module_path = sys.argv[1]
output_directory = sys.argv[2]
log_file_path = None if len(sys.argv) < 4 else sys.argv[3]

# Open up our message output directory to get our protobuf types
sys.path.append(message_module_path)
# Load all of the protobuf files as modules to use
for dir_name, subdir, files in os.walk(message_module_path):
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

if os.path.isdir(os.path.join(output_directory, 'message')):
    shutil.rmtree(os.path.join(output_directory, 'message'))

files = {}
for message in messages:
    current = files
    fqn = message[0].replace('protobuf.', '').split('.')

    for element in fqn:
        if element not in current:
            current[element] = {}
        current = current[element]
    current['file'] = message[1].replace('.proto', '.h')

with datasink(log_file_path, 'a') as log_file:
    recurse(files, output_directory, log_file)
