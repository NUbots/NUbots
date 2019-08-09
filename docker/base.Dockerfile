FROM alpine:edge

# Install user and build tools
# We need the testing repository for some packages
RUN apk update && apk add --no-cache \
    alpine-sdk \
    cmake

# Add a useful utility and set up host aliases
COPY "files/find_robot_hosts.sh" "/usr/bin/find_robot_hosts.sh"
COPY "files/hosts" "/etc/hosts"

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

# Copy across the abuild configuration
COPY --chown="root:root" "files/abuild.conf" "/etc/abuild.conf"

# Copy across the nuinstall script that wraps abuild
COPY --chown="nubots:nubots" "files/nuinstall" "/usr/local/bin/nuinstall"

# Swap to using the nubots user from now on
USER nubots
WORKDIR /home/nubots
