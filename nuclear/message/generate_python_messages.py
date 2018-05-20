#!/usr/bin/env python3
import sys
import os
import pkgutil
import google.protobuf.message
from generator.textutil import dedent, indent


def recurse(messages, key='message', depth=0):
    template = dedent("""\
        class {name}:
            def include_path(self): return None
        """)
    sub_messages = []
    for message in messages[key]:
        sub_messages.append(recurse(messages[key], message, depth))
    return '\n'.join([indent(template.format(name=key), depth), indent('\n'.join(sub_messages), depth + 4)])


# Open up our message output directory to get our protobuf types
sys.path.append(sys.argv[1])
# Load all of the protobuf files as modules to use
for dir_name, subdir, files in os.walk(sys.argv[1]):
    modules = pkgutil.iter_modules(path=[dir_name])
    for loader, module_name, ispkg in modules:
        if module_name.endswith('pb2'):
            # Load our protobuf module
            module = loader.find_module(module_name).load_module(module_name)
messages = {}
# Now that we've imported them all get all the subclasses of protobuf message
for message in google.protobuf.message.Message.__subclasses__():
    # Work out our original protobuf type
    pb_type = message.DESCRIPTOR.full_name.split('.')[1:]
    # Ignore messages that aren't ours
    if len(pb_type) > 0 and not pb_type[0] == 'protobuf':
        sub_message = messages
        # Generate a dict with all the messages nested hierarchically
        for i in range(len(pb_type)):
            if pb_type[i] not in sub_message:
                sub_message[pb_type[i]] = {}
            sub_message = sub_message[pb_type[i]]
with open(sys.argv[2], 'w') as f:
    f.write(recurse(messages, list(messages.keys())[0]))
