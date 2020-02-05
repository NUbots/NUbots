#!/usr/bin/env python3

import enum
import gzip
import os
import struct

from .protobuf_types import MessageTypes


class NBSPacket:
    def __init__(self, type, timestamp, msg, raw):
        self.type = type
        self.timestamp = timestamp
        self.msg = msg
        self.raw = raw


class Decoder:
    def __init__(self, path, *paths):

        # The paths we have yet to read
        self._paths = list(paths)
        self._current_file = gzip.open(path, "rb") if path.endswith("nbz") or path.endswith(".gz") else open(path, "rb")
        self._current_path = path

        # Tracking of how much we have read vs how much there is to read
        self._bytes_read = 0
        self._total_size = os.path.getsize(path)

        # Descriptions of states and packets
        self._state = enum.Enum("State", "header_lock_0 header_lock_1 header_lock_2 packet")

        # Work out the total length of all files
        for p in paths:
            self._total_size += os.path.getsize(p)

    def __len__(self):
        return self._total_size

    def bytes_read(self):
        return self._bytes_read

    def current_path(self):
        return self._current_path

    def __iter__(self):
        return self

    def __next__(self):
        # NBS File Format:
        # 3 Bytes - NUClear radiation symbol header, useful for synchronisation when attaching to an existing stream
        # 4 Bytes - The remaining packet length i.e. 16 bytes + N payload bytes
        # 8 Bytes - 64bit timestamp in microseconds. Note: this is not necessarily a unix timestamp
        # 8 Bytes - 64bit bit hash of the message type
        # N bytes - The binary packet payload
        sync_state = self._state.header_lock_0
        bytes_read = self._current_file.tell()

        # While we can read a header
        while True:
            # If we reach sync state 3 we are ready to read a packet
            if sync_state == self._state.packet:

                sync_state = self._state.header_lock_0

                # Read our header
                size, timestamp, type_hash = struct.unpack("<IQQ", self._current_file.read(20))

                # Read our payload
                payload = self._current_file.read(size - 16)

                # If we know how to parse this type, parse it
                if type_hash in MessageTypes:
                    # Yield a message
                    try:
                        # Build the packet
                        packet = NBSPacket(
                            type=MessageTypes[type_hash].name,
                            timestamp=timestamp,
                            msg=MessageTypes[type_hash].type.FromString(payload),
                            raw=payload,
                        )

                        # Update bytes read
                        self._bytes_read += self._current_file.tell() - bytes_read

                        # Return the packet
                        return packet
                    except:
                        pass
            else:
                c = self._current_file.read(1)

                # No bytes to read we are EOF
                # Therefore we jump to the next file and reset our state
                if len(c) == 0:
                    # Update bytes read
                    self._bytes_read += self._current_file.tell() - bytes_read
                    bytes_read = 0

                    # Close this file
                    self._current_file.close()

                    # If we have files left to read, jump to the next one
                    if len(self._paths) > 0:
                        path = self._paths.pop(0)
                        sync_state = self._state.header_lock_0
                        self._current_file.close()
                        self._current_file = (
                            gzip.open(path, "rb") if path.endswith("nbz") or path.endswith(".gz") else open(path, "rb")
                        )
                        self._current_path = path

                    # We're done
                    else:
                        raise StopIteration()

                # We can jump straight to lock1 from any state
                if c == b"\xE2":
                    sync_state = self._state.header_lock_1
                elif sync_state == self._state.header_lock_1 and c == b"\x98":
                    sync_state = self._state.header_lock_2
                elif sync_state == self._state.header_lock_2 and c == b"\xA2":
                    sync_state = self._state.packet
