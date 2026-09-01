#!/bin/bash

URL="https://github.com/bit-bots/TORSO_21_dataset/archive/refs/heads/master.zip"
FOLDER="TORSO_21_dataset-master/data"
DEST="$(pwd)/data"

if [ -d "$DEST" ]; then
    echo "Dataset already exists: $DEST"
    exit 0
fi

TMP=$(mktemp)

curl -L "$URL" -o "$TMP"
unzip -q "$TMP" "$FOLDER/*" -d .

mkdir -p "$DEST"
mv "$FOLDER"/* "$DEST/"

rm -rf "TORSO_21_dataset-master" "$TMP"

echo "Dataset downloaded to: $DEST"