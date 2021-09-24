voice2json
==========

## Usage

To test:

```bash
espeak -w audio.wav "Robot please escort the tallest person in the living room to the exit"
KALDI_DIR= python -m voice2json --profile en transcribe-wav audio.wav | # speech to text
python -m voice2json --profile en recognize-intent |                    # text to intent
jq                                                                      # pretty print
```

The sentences.ini file has been created for targeting the general purpose command set and will need to be modified for the enhanced general purpose command set.

Mount your docker container so that it can access your nubots folder...

```bash
docker run -it --rm --mount type=bind,source=/Users/realideasman/Developer/NUbots/,target=/home/nubots/NUbots nuhear:latest
```

If you want to create the audio data for testing:

```bash
while IFS= read -r line; do echo $line | espeak -w "audio-$i.wav"; i=$((i+1)); done < ../sentences.txt
```

Make sure you do it inside of docker.

If you have a directory of audio files then you can transcribe all of it:

```bash
KALDI_DIR= python -m voice2json -p en transcribe-wav audio/*.wav > results.json
cat results.json | python -m voice2json -p en recognize-intent > intention-results.jsonl
```
