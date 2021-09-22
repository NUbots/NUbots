Microphone
==========

## Description
This module spawns the "voice2json" program as a subprocess and waits until 

## Usage


## Emits
This module emits a "SpeechIntentMessage" when it has successfully 

struct SpeechIntentMessage {
    std::string text;
    std::string intent;
    std::vector<Slot> slots;
    float confidence;
};


## Dependencies

