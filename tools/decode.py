#!/usr/bin/env python3

import mmh3
import struct
import sys
import os
import re
import b
import pkgutil
import google.protobuf.message
from PIL import Image
from google.protobuf.json_format import MessageToJson

def register(command):

    # Install help
    command.help = 'Decode an nbs file into a series of json objects'

    # Drone arguments
    command.add_argument('file'
        , metavar='file'
        , help='The file to decode into a strint')

def run(file, **kwargs):

    # Open up our message output directory to get our protobuf types
    shared_path = os.path.join(b.cmake_cache[b.cmake_cache["CMAKE_PROJECT_NAME"] + '_BINARY_DIR'], 'shared')

    sys.path.append(shared_path)

    # Load all of the protobuf files as modules to use
    for dir_name, subdir, files in os.walk(shared_path):
        modules = pkgutil.iter_modules(path=[dir_name])
        for loader, module_name, ispkg in modules:
            if module_name.endswith('pb2'):

                # Load our protobuf module
                module = loader.find_module(module_name).load_module(module_name)

    parsers = {}

    # Now that we've imported them all get all the subclasses of protobuf message
    for message in google.protobuf.message.Message.__subclasses__():

        # Work out our original protobuf type
        pb_type = message.__module__.split('.')[:-1]
        pb_type.append(message.__name__)
        pb_type = '.'.join(pb_type).encode('utf-8')
        pb_hash = mmh3.hash_bytes(pb_type, 0x4e55436c, True)

        parsers[pb_hash] = (pb_type, message)

    # Now open the passed file
    with open(file, 'rb') as f:

        rH264 = open('r.h264', 'wb')
        g1H264 = open('g1.h264', 'wb')
        g2H264 = open('g2.h264', 'wb')
        bH264 = open('b.h264', 'wb')

        imgno = 0
        # While we can read a header
        while len(f.read(3)) == 3:

            # Read our size
            size = struct.unpack('<I', f.read(4))[0]

            # Read our payload
            payload = f.read(size)

            # Read our timestamp
            timestamp = struct.unpack('<Q', payload[:8])[0]

            # Read our hash
            type_hash = payload[8:24]

            # If we know how to parse this type, parse it
            if type_hash in parsers:
                msg = parsers[type_hash][1].FromString(payload[24:])

                if parsers[type_hash][0] == b'message.vision.Image':

                    img = Image.new('RGB', (int(msg.dimensions.x / 2), int(msg.dimensions.y / 2)))
                    pixels = img.load()

                    for x in range(0, msg.dimensions.x, 2):
                        for y in range(0, msg.dimensions.y, 2):
                            red  = (x + 0, y + 0)
                            green1 = (x + 1, y + 0)
                            green2 = (x + 0, y + 1)
                            blue  = (x + 1, y + 1)

                            red  =   msg.payload.v[red [1] * msg.dimensions.x + red [0]]
                            green1 = msg.payload.v[green1[1] * msg.dimensions.x + green1[0]]
                            green2 = msg.payload.v[green2[1] * msg.dimensions.x + green2[0]]
                            blue  =  msg.payload.v[blue [1] * msg.dimensions.x + blue [0]]

                            pixels[int(x/2), int(y/2)] = (red, int((green1 + green2) / 2), blue)

                    img.save('frame_{}.png'.format(imgno))
                    imgno += 1

                elif parsers[type_hash][0] == b'message.vision.CompressedImage':
                    rH264.write(msg.payloads[0])
                    g1H264.write(msg.payloads[1])
                    g2H264.write(msg.payloads[2])
                    bH264.write(msg.payloads[3])

                else:
                    out = re.sub(r'\s+', ' ', MessageToJson(msg, True))
                    out = '{{ "type": "{}", "timestamp": {}, "data": {} }}'.format(parsers[type_hash][0].decode('utf-8'), timestamp, out)
                    # Print as a json object
                    print(out)

