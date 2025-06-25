#!/bin/bash

ALLOWLIST_FILE="usb_allowlist"

# Extract vendor:product pairs from allowlist, ignoring comments and blank lines, normalize to lowercase
allowlist=$(grep -Eo '^[0-9a-fA-F]{4}:[0-9a-fA-F]{4}' "$ALLOWLIST_FILE" | tr '[:upper:]' '[:lower:]')

# Get vendor:product pairs from lsusb output, normalized to lowercase
connected_devices=$(lsusb | awk '{print tolower($6)}')

# Check each connected device against the allowlist
for device in $connected_devices; do
    if ! grep -qx "$device" <<< "$allowlist"; then
        exit 1
    fi
done

exit 0
