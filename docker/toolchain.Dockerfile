FROM nubots:base
ARG platform

# Copy across the specific toolchain files for this image
COPY --chown=nubots:nubots toolchain/${platform}.cmake /usr/local/toolchain.cmake

# TODO use the tools built in nubots:base to build this specific toolchain
# TODO use ${platform} to pull the specific options

COPY --chown=nubots:nubots package/zlib/APKBUILD /home/nubots/zlib/APKBUILD
RUN cd /home/nubots/zlib \
    && abuild checksum \
    && abuild -r \
    && sudo apk add --no-cache /home/nubots/packages/nubots/x86_64/nubots-zlib-1.2.11-r1.apk \
    && rm -rf /home/nubots/packages \
    && rm -rf /home/nubots/package_sources \
    && rm -rf /home/nubots/zlib \
    && sudo chown -R nubots:nubots /usr/local
