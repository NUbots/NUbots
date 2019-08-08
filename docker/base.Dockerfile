FROM alpine:edge

# Install user and build tools
# We need the testing repository for some packages
RUN apk update && apk add --no-cache \
    alpine-sdk \
    cmake

# Add a useful utility and set up host aliases
COPY "files/find_robot_hosts.sh" "/usr/bin/find_robot_hosts.sh"
COPY "files/hosts" "/etc/hosts"
RUN chmod 755 "/usr/bin/find_robot_hosts.sh" && chmod 644 "/etc/hosts"

# Set up user and set password and allow passwordless sudo
RUN adduser -D nubots \
    && addgroup nubots abuild \
    && addgroup nubots wheel \
    && echo "nubots: " | chpasswd \
    && echo "%wheel ALL=(ALL) NOPASSWD: ALL" | tee -a /etc/sudoers \
    && visudo -c

# Setup /usr/local owned by nubots
RUN chown -R nubots:nubots /usr/local

# Add ssh key for sshing into the robot
COPY --chown="nubots:nubots" "files/id_rsa" "/home/nubots/.ssh/id_rsa"
COPY --chown="nubots:nubots" "files/id_rsa.pub" "/home/nubots/.ssh/id_rsa.pub"
COPY --chown="nubots:nubots" "files/ssh_config" "/home/nubots/.ssh/ssh_config"
RUN chmod 600 "/home/nubots/.ssh/id_rsa" \
    && chmod 600 "/home/nubots/.ssh/id_rsa.pub"

# Swap to using the nubots user from now on
USER nubots
WORKDIR /home/nubots

# Create key for use with abuild and fix abuild.conf
RUN abuild-keygen -a -i -n \
    && sed 's/SRCDEST=.*/SRCDEST=$HOME\/package_sources/g' \
    && sed 's/REPODEST=.*/REPODEST=$HOME\/packages/g' \
    && sed 's/#PACKAGER=.*/PACKAGER="NUbots <nubots@newcastle.edu.au>"/g' \
    && sed 's/#MAINTAINER=.*/MAINTAINER=$PACKAGER/g' \
    && sed 's/ERROR_CLEANUP=.*/ERROR_CLEANUP="srcdir bldroot pkgdir deps"/g'
