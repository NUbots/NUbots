#!/bin/sh

install-make-from-source $1 \
    CROSS=1 \
    BINARY=64 \
    SMP=1 \
    NUM_THREADS=4 \
    TARGET=HASWELL \
    USE_THREAD=1 \
    NO_SHARED=0 \
    NO_STATIC=0
