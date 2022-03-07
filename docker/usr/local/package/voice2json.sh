#!/bin/bash

# Exit immediately on error
set -e

# Set installation prefix, default to /usr/local unless an external power says otherwise
PREFIX=${PREFIX:-"/usr/local"}

BUILD_FOLDER="/var/tmp/build"

GIT_COMMIT="$1"

# Pull in the toolchain arguments
. /usr/local/toolchain.sh

# Setup the temporary build directories
mkdir -p "${BUILD_FOLDER}"
cd "${BUILD_FOLDER}"

# Download the source code

# Download the provided git commit and check it out.
git clone https://github.com/synesthesiam/voice2json voice2json
cd voice2json
git checkout "${GIT_COMMIT}"

echo "Installing voice2json... by hand."

# The ./configure script does not work on Arch Linux, perform the equivalent substitutions manually.
cat <<EOF | sed -f - setup.py.in > setup.py
/@ENABLE_POCKETSPHINX@/s//no/
/@ENABLE_KALDI@/s//yes/
/@ENABLE_DEEPSPEECH@/s//no/
/@PACKAGE_NAME@/s//voice2json/
/@PACKAGE_VERSION@/s//${GIT_COMMIT}/
/@PACKAGE_BUGREPORT@/s//mike@rhasspy.org/
EOF

# Install voice2json under /usr/local/lib/python3.9/site-packages
sudo pip install .

# Install data files that otherwise would have been installed by the makefile.
sudo mkdir -p "${PREFIX}/share/voice2json"
sudo cp --recursive --target-directory="${PREFIX}/share/voice2json" "VERSION" "README.md" "LICENSE" "etc/"

cat <<EOF > /usr/local/bin/voice2json
#!/bin/sh

export PATH="\$PATH:\$(echo /usr/local/kaldi/src/*bin | sed 's/ /:/g')"
export KALDI_DIR=\${KALDI_DIR-/usr/local/kaldi}
export voice2json_dir=/usr/local/share/voice2json
export XDG_DATA_HOME=/usr/local/share/voice2json

python -m voice2json --profile en-us_kaldi-zamia "\$@"
EOF
chmod +x /usr/local/bin/voice2json

echo "Downloading speech recognition models..."

voice2json download-profile
voice2json train-profile

# Clean up the build directory.
rm -rf "${BUILD_FOLDER}"
