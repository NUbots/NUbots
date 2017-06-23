#!/usr/bin/env python3

import pickle
import gzip
import struct
import sys
import os
import re
import b
import pkgutil
import google.protobuf.message
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
        pb_hash = '' # TODO swap to using XXHASH
        # pb_hash = mmh3.hash_bytes(pb_type, 0x4e55436c, True)

        parsers[pb_hash] = (pb_type, message)

    servo_health = {}

    # Now open the passed file
    with gzip.open(file, 'rb') if file.endswith('nbz') or file.endswith('.gz') else open(file, 'rb') as f:
        with gzip.open('output.pickle', 'wb') as of:

            prev_goal_position = None
            state = None
            c_pass = []

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

                    if parsers[type_hash][0] == b'message.support.ServoHealthTestData':
                        # Read our servo
                        servo = msg.sensors.servo.rAnkleRoll

                        if prev_goal_position == None:
                            prev_goal_position = round(servo.goalPosition)
                        elif prev_goal_position != round(servo.goalPosition):
                            # Changing motion
                            if state != None:
                                pickle.dump((state, c_pass), of, protocol=pickle.HIGHEST_PROTOCOL)
                                c_pass = []

                            if prev_goal_position == 0:
                                # Second half
                                if round(servo.goalPosition) > prev_goal_position:
                                    # increasing
                                    state = 'inc2stop'
                                else:
                                    # decreasing
                                    state = 'dec2stop'
                            elif round(servo.goalPosition) == 0:
                                # First half
                                if round(servo.goalPosition) > prev_goal_position:
                                    # increasing
                                    state = 'inc1stop'
                                else:
                                    # decreasing
                                    state = 'dec1stop'
                            elif round(servo.goalPosition) > prev_goal_position:
                                # increasing
                                state = 'inc'
                            else:
                                # decreasing
                                state = 'dec'

                            prev_goal_position = round(servo.goalPosition)

                        c_pass.append({
                            'error_flags': servo.errorFlags,
                            'present_position': servo.presentPosition,
                            'goal_position': servo.goalPosition,
                            'present_speed': servo.presentSpeed,
                            'goal_speed': servo.movingSpeed,
                            'load': servo.load,
                            'voltage': servo.voltage,
                            'temperature': servo.temperature
                        })

                    else:
                        out = re.sub(r'\s+', ' ', MessageToJson(msg, True))
                        out = '{{ "type": "{}", "timestamp": {}, "data": {} }}'.format(parsers[type_hash][0].decode('utf-8'), timestamp, out)
                        # Print as a json object
                        print(out)

