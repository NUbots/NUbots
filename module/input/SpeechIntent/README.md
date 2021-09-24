SpeechIntent
==========

## Description
This purpose of this module is to recognize the intention of speech audio data. The audio source can come from a wav file or as an audio stream (from a microphone). The results of speech intention recognition will be emited as a message.
This module spawns the "voice2json" program as a subprocess and waits until an intention is recognised, upon which
a messages is emitted into nuclear.

## Usage

Settings are configured in the "SpeechIntent.yaml" settings file. 

When used with the ./b tool e.g. "./b run speechintent" several command line arguments are available:
- "file" - this argument 

To use this as another module:

Add "input::SpeechIntent" to your .role file for your module.
Within the constructor of your module add the following "on Trigger" statement:

```c++
    on<Trigger<SpeechIntentMsg>>().then([this](const SpeechIntentMsg& msg) {
        // ...
    });
```

This "on Trigger" statement will wait for "SpeechIntentMsg" messages (described further in the "Emits" section). The "then" function registers the callback which processes this message. Inside the body of the callback you can write your own processing code based on your needs.

As an example, the following code will log the fields of the message. (Make sure the log level is set in the SpeechIntent.yaml settings file)

```c++
    on<Trigger<SpeechIntentMsg>>().then([this](const SpeechIntentMsg& msg) {
        log(fmt::format("text = {}", msg.text.data()));
        log(fmt::format("intent = {}", msg.intent.data()));
        log("slots");
        for (Slot slot : msg.slots) {
            log(fmt::format("{} = {}", slot.name.data(), slot.value.data()));
        }
    });
```

When running in "transcribe_mode: file" the module will wait for an audio wav file. For example, to send an audio wav file:
```c++
    std::string filename = "/home/nubots/nuhear/audio/voice_001.wav";
    auto msg = std::make_unique<SpeechInputRecognizeWavFile>(SpeechInputRecognizeWavFile{filename});
    emit(std::move(msg));
```

By default the module will output "SpeechIntentMsg" messages. To control whether or not the module outputs a message you can send a "SpeechInputSetOutputMsg". For example, to disable the output send:
```c++
    auto disable_output_msg = std::make_unique<SpeechInputSetOutputMsg>({false});
    emit(std::move(disable_output_msg));
```

to enable the output send:
```c++
    auto enable_output_msg = std::make_unique<SpeechInputSetOutputMsg>({true});
    emit(std::move(enable_output_msg));
```

## Emits
This module emits a "SpeechIntentMessage" when it has recognized the intent of the provided audio data.  (The defintion of this message can be found in SpeechIntent.hpp). This message contains some fields "text", "intent" and "slots".

```c++
    struct Slot {
        std::string name;
        std::string value;
    };
    struct SpeechIntentMessage {
        std::string text;
        std::string intent;
        std::vector<Slot> slots;
        float confidence;
    };
```

The "text" field describes the raw text as recognized by speech recognition. The "intent" field describe the detected intention of the speech data - . The "slots" fields describes supplementary information about the given intent of the speech. A slot is a name, value pair. For example, .

## Dependencies
- Voice2json - this module is depedant on the voice2json project - which is a . This process spawns voice2json via python.
- This module is dependant on the "extension::FileWatcher" and "support::logging::ConsoleLogHandler" modules for logging
