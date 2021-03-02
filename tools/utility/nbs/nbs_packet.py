#!/usr/bin/env python3


class NBSPacket:
    def __init__(self, type, index_timestamp, emit_timestamp, msg, raw):
        self.type = type
        self.index_timestamp = index_timestamp
        self.emit_timestamp = emit_timestamp
        self.msg = msg
        self.raw = raw
