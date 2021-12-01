# Demonstration Script

# Part 1

# Pull the image and make it the selected one.
docker pull nubots/nubots:pull-request-737

# ./b script will now use this image.
docker tag nubots/nubots:pull-request-737 nubots:generic
docker tag nubots/nubots:pull-request-737 nubots:selected

# Record 3s of audio on macOS
ffmpeg -f avfoundation -i ":0" -t 3 audio/clean_up_the_bed_room.wav
# wait a little bit then say "clean up the bedroom"

# Play the recorded track
afplay audio/clean_up_the_bed_room.wav

# Start the docker container
./b shell

# Run transcribe-wav on "clean up the bedroom"
KALDI_DIR= python -m \
    voice2json --profile en transcribe-wav audio/clean_up_the_bed_room.wav |
    python -m \
    voice2json --profile en recognize-intent |
    jq -C | less -R

# Examine voice2json output

# Leave docker container.
exit

# Part 2

# Build our module. This might take a while.
./b build speechintent

# Process a single file (useful for automated testing):
./b run speechintent -- file /home/nubots/NUbots/audio/clean_up_the_bed_room.wav
# note: press ctrl-c then any key to exit

# Interactive mode (useful for iterating):
./b run speechintent -- cli

# Then type:
# file /home/nubots/NUbots/audio/clean_up_the_bed_room.wav

# Part 3

# Record yourself saying "make me a sandwich".
ffmpeg -f avfoundation -i ":0" -t 3 audio/make_me_a_sandwich.wav
# Record yourself saying "sudo make me a sandwich".
ffmpeg -f avfoundation -i ":0" -t 3 audio/sudo_make_me_a_sandwich.wav

# Enter the docker container.
./b shell

# Use your text editor to write to sentences.ini file.
vim /home/nubots/.local/share/voice2json/en-us_kaldi-zamia/sentences.ini

# Write the example sentences.ini file to where voice2json looks for it.
cat > /home/nubots/.local/share/voice2json/en-us_kaldi-zamia/sentences.ini <<EOF
[Sandwhich]

# sue doe => sudo
((sue doe):true | :false){asked_with_sudo} make me a sandwich
EOF

# Retrain the profile.
python -m voice2json -p en train-profile

# Run speech recognition and intention recognition for "make me a sandwich".
KALDI_DIR= python -m \
    voice2json --profile en transcribe-wav audio/make_me_a_sandwich.wav |
    python -m \
    voice2json --profile en recognize-intent |
    jq -C | less -R

# Run speech recognition and intention recognition for "sudo make me a sandwich".
KALDI_DIR= python -m \
    voice2json --profile en transcribe-wav audio/sudo_make_me_a_sandwich.wav |
    python -m \
    voice2json --profile en recognize-intent |
    jq -C | less -R
