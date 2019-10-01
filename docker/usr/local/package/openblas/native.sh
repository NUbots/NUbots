#!/bin/sh

install-make-from-source $1 \
    SMP=1 \
    USE_THREAD=1 \
    NO_SHARED=0 \
    NO_STATIC=0
