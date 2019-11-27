#!/usr/bin/env python3


def fourcc(c):
    return ord(c[3]) << 24 | ord(c[2]) << 16 | ord(c[1]) << 8 | ord(c[0])
