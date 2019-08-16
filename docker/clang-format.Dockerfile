FROM alpine:edge

# Get clang-format
RUN apk update && apk add --no-cache clang colordiff

# Add a NUbots user
RUN addgroup -S nubots && adduser -S nubots -G nubots
USER nubots

# Create the home directory owned by nubots
RUN mkdir -p /home/nubots && chown -R nubots:nubots /home/nubots

WORKDIR /home/nubots
