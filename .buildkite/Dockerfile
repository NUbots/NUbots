FROM ubuntu:16.04

ENV DEBIAN_FRONTEND=noninteractive

# Install packages needed for the build scripts
RUN apt-get update \
    && apt-get install -y sudo software-properties-common git wget clang-format-6.0 colordiff parallel \
    && apt-add-repository -y ppa:rael-gc/rvm \
    && apt-get update \
    && apt-get install -y rvm \
    && apt-get autoremove --purge

# Add puppet and build scripts to the image
RUN mkdir -p /NUbots-build/toolchain/
ADD ./puppet /NUbots-build/puppet/
ADD ./.buildkite /NUbots-build/.buildkite/

# Install the NUbots toolchain
RUN DOCKER_BUILD_DIR=/NUbots-build /NUbots-build/.buildkite/scripts/install_toolchain.sh

# Pip install some extra packages for formatting
RUN pip3 install cmake-format

# Remove extraneous files from the toolchain install
RUN rm -rf /NUbots-build \
    && apt-get autoremove --purge

# Create the directory that will be used for checkouts and builds
RUN mkdir -p /code/NUbots

# Create the nubots user and group with id 1000.
# Used to login and run commands in the container.
RUN groupadd -g 1000 nubots \
    && useradd -r -u 1000 -g nubots nubots

# Give the nubots user ownership of the /code directory,
# and give all users read, write, and execute permissions
# Used to fix permission issues
RUN chown -R nubots:nubots /code \
    && chmod a+rwx -R /code

# Use the nubots user if no user is specified with `docker run`
USER nubots

# Run bash if no command is specified with `docker run`
CMD ["bash"]
