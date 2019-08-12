FROM alpine:edge
RUN apk update && apk add --no-cache clang

# Add a NUbots user
RUN addgroup -S nubots && adduser -S nubots -G nubots
USER nubots
WORKDIR /home/nubots
