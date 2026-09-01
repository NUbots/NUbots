#! /bin/bash

URL="$1"
NAME="$2"
DEST="$(pwd)/models/$NAME"

if [ -d "$DEST" ]; then
    echo "Model already exists: $DEST"
    exit 0
fi

git clone "$URL" "$DEST"

echo "Model: $NAME downloaded to: $DEST"