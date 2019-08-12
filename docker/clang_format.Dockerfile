FROM alpine:edge
RUN apk update && apk add --no-cache clang
