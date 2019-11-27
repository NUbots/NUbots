#!/usr/bin/env python3


def fourcc(c):
    return ord(c[3]) << 24 | ord(c[2]) << 16 | ord(c[1]) << 8 | ord(c[0])


def fourcc_to_string(c):
    return "".join([chr((c >> 0) & 0xFF), chr((c >> 8) & 0xFF), chr((c >> 16) & 0xFF), chr((c >> 24) & 0xFF)])
