#!/bin/bash

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
if [ ${#unauthorized[@]} -ne 0 ]; then
    for dev in "${unauthorized[@]}"; do
        echo "Connected USB device not in allowlist: $dev" >&2
    done
    exit 1
fi

exit 0
