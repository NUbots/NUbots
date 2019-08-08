FROM alpine:edge

# Install user and build tools
# We need the testing repository for some packages
RUN apk update && apk add --no-cache \
    build-base \
    alpine-sdk \
    cmake

# Add a useful utility and set up host aliases
COPY "files/find_robot_hosts.sh" "/usr/bin/find_robot_hosts.sh"
COPY "files/hosts" "/etc/hosts"
RUN chmod 755 "/usr/bin/find_robot_hosts.sh" && chmod 644 "/etc/hosts"

# Set up user
RUN addgroup nubots && adduser -D -G nubots -G abuild nubots

# Setup /usr/local owned by nubots
RUN chown -R nubots:nubots /usr/local

# Add ssh key for sshing into the robot
COPY --chown="nubots:nubots" "files/id_rsa" "/home/nubots/.ssh/id_rsa"
COPY --chown="nubots:nubots" "files/id_rsa.pub" "/home/nubots/.ssh/id_rsa.pub"
COPY --chown="nubots:nubots" "files/ssh_config" "/home/nubots/.ssh/ssh_config"
RUN chmod 600 "/home/nubots/.ssh/id_rsa" \
    && chmod 600 "/home/nubots/.ssh/id_rsa.pub"

# Copy across the same keys for use by abuild
COPY --chown="root:root" "files/id_rsa.pub" "/etc/apk/keys/nubots@newcastle.edu.au.rsa.pub"
COPY --chown="nubots:nubots" "files/abuild.conf" "/home/nubots/.abuild/abuild.conf"

# Swap to using the nubots user from now on
USER nubots
WORKDIR /home/nubots
