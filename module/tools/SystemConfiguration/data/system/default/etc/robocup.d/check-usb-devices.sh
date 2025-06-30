#!/bin/bash
##
## MIT License
##
## Copyright (c) 2025 NUbots
##
## This file is part of the NUbots codebase.
## See https://github.com/NUbots/NUbots for further info.
##
## Permission is hereby granted, free of charge, to any person obtaining a copy
## of this software and associated documentation files (the "Software"), to deal
## in the Software without restriction, including without limitation the rights
## to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
## copies of the Software, and to permit persons to whom the Software is
## furnished to do so, subject to the following conditions:
##
## The above copyright notice and this permission notice shall be included in all
## copies or substantial portions of the Software.
##
## THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
## AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
## OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
## SOFTWARE.
##

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
ALLOWLIST_FILE="$SCRIPT_DIR/usb_allowlist"

# Extract vendor:product pairs from allowlist, ignoring comments and blank lines, normalise to lowercase
allowlist=$(grep -Eo '^[0-9a-fA-F]{4}:[0-9a-fA-F]{4}' "$ALLOWLIST_FILE" | tr '[:upper:]' '[:lower:]')

# Track unauthorized devices
unauthorised=()

# Loop over each line of lsusb
while IFS= read -r line; do
    id=$(awk '{print tolower($6)}' <<< "$line")
    if ! grep -qx "$id" <<< "$allowlist"; then
        desc=$(sed 's/.*\(ID [^ ]\+ [^ ]\+.*\)/\1/' <<< "$line")
        unauthorised+=("$desc")
    fi
done <<< "$(lsusb)"

# Log and exit if any unauthorised devices found
if [ ${#unauthorised[@]} -ne 0 ]; then
    for dev in "${unauthorised[@]}"; do
        echo "Connected USB device not in allowlist: $dev" >&2
    done
    exit 1
fi

exit 0
