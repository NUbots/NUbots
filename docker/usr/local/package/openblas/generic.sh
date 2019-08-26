#!/bin/sh

install-make-from-source $1 \
    BINARY=64 \
    SMP=1 \
    NUM_THREADS=4 \
    DYNAMIC_ARCH=1 \
    TARGET=GENERIC \
    USE_THREAD=1  \
    NO_SHARED=0 \
    NO_STATIC=0
