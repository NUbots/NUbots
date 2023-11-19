#!/bin/bash

#
# MIT License
#
# Copyright (c) 2021 NUbots
#
# This file is part of the NUbots codebase.
# See https://github.com/NUbots/NUbots for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

# Script for deleting tags with the DockerHub REST API
# Method adapted from here
# https://devopsheaven.com/docker/dockerhub/2018/04/09/delete-docker-image-tag-dockerhub.html

# Exit immediately on error
set -e

# DOCKERHUB_USERNAME, DOCKERHUB_PASSWORD, TAG need to be passed in as environment variables
if [[ ! -v DOCKERHUB_USERNAME ]]; then
    echo "DOCKERHUB_USERNAME not set. Cannot delete tag without valid credentials."
    exit 1
elif [[ ! -v DOCKERHUB_PASSWORD ]]; then
    echo "DOCKERHUB_PASSWORD not set. Cannot delete tag without valid credentials."
    exit 1
elif [[ ! -v TAG ]]; then
    echo "TAG not set. Cannot delete tag without a tag name to delete."
    exit 1
fi

# These are always the same
ORGANIZATION="nubots"
IMAGE="nubots"

login_json() {
cat <<EOF
{
  "username": "$DOCKERHUB_USERNAME",
  "password": "$DOCKERHUB_PASSWORD"
}
EOF
}

TOKEN=$(curl -s -H "Content-Type: application/json" -X POST -d "$(login_json)" "https://hub.docker.com/v2/users/login/" | jq -r .token)

# We capture the HTTP status code making the API call
HTTP_CODE=$(curl "https://hub.docker.com/v2/repositories/${ORGANIZATION}/${IMAGE}/tags/${TAG}/" \
        --silent --output /dev/stderr \
        -X DELETE \
        -H "Authorization: JWT ${TOKEN}" \
        -w "%{http_code}")

# status code 204 means success
if [[ $HTTP_CODE -ne 204 ]]; then
    echo "API call not accepted. HTTP status code = ${HTTP_CODE}."
    exit 2
fi
