FROM alpine:3.10.1

# We need a compiler to bootstrap the process and wget to download files
# RUN apk add --update build-base wget
WORKDIR /build

COPY scripts/binutils.sh /build/scripts/binutils.sh
RUN /build/scripts/binutils.sh 2.32
