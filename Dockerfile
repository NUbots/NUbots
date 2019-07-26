FROM ubuntu:16.04

ENV DEBIAN_FRONTEND=noninteractive

# Install packages needed for the build scripts
RUN apt-get update \
    && apt-get install -y sudo software-properties-common git wget \
    && apt-add-repository -y ppa:rael-gc/rvm \
    && apt-get update \
    && apt-get install -y rvm

# Add puppet and build scripts to the image
RUN mkdir -p /NUbots-build/toolchain/
ADD ./puppet /NUbots-build/puppet/
ADD ./.travis /NUbots-build/.travis/

# Install clang-format and the NUbots toolchain
RUN /NUbots-build/.travis/install_clang-format.sh
RUN TRAVIS_BUILD_DIR=/NUbots-build /NUbots-build/.travis/install_toolchain.sh

# Run bash if no command is given with `docker run`
CMD ["bash"]
