FROM nubots:base
ARG platform

# Copy across the specific toolchain files for this image
COPY --chown=nubots:nubots toolchain/${platform}.cmake /usr/local/toolchain.cmake
COPY --chown=nubots:nubots toolchain/${platform}.sh /usr/local/toolchain.sh

# TODO use the tools built in nubots:base to build this specific toolchain
# TODO use ${platform} to pull the specific options

COPY --chown=nubots:nubots package/zlib/APKBUILD /usr/local/package/zlib/APKBUILD
RUN nuinstall /usr/local/package/zlib
