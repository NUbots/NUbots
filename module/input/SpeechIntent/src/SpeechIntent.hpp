#ifndef MODULE_INPUT_SPEECHINTENT_HPP
#define MODULE_INPUT_SPEECHINTENT_HPP

#include <nuclear>
#include <spawn.h>

namespace module::input {
    static constexpr int read_buffer_size = 0x1000;

    /**
     * @brief @c SpeechInputSetOutputMsg is used to enable or disable microphone processing.
     */
    struct SpeechInputSetOutputMsg {
        /**
         * @brief If @c enabled is true then microphone processing will be enabled for the module, otherwise it will be
         *          disabled.
         */
        bool enabled;
    };

    /**
     * @brief @c SpeechInputRecognizeWavFile is used for testing the module's ability to recognise an intention given an
     *          wav audio recording.
     */
    struct SpeechInputRecognizeWavFile {
        /**
         * @brief This field should be a full path to a WAV file.
         */
        std::string filename;
    };

    /**
     * @brief A private data structure used to maintain a references to a subprocess's identifiers.
     */
    struct SpawnedProcess {
        /**
         * @brief The standard output stream of the spawned subprocess.
         * @details This file is open for reading within the parent process space.
         */
        int stdout;
        /**
         * @brief The standard error stream of the spawned subprocess.
         * @details This file is open for reading within the parent process space.
         */
        int stderr;
        /**
         * @brief The standard input stream of the spawned subprocess.
         * @details This file is open for writing within the parent process space.
         */
        int stdin;
        /**
         * @brief The process identifier of the subprocess.
         */
        pid_t pid;
        /**
         * @brief The executable that the subprocess is running.
         * @details This variable has the same semantics as the second parameter in @c posix_spawnp() .
         */
        std::string process_name;
    };

    /**
     * @brief A slot represents a named substituted value within the sentence.
     * @details Slots can represent slot references, slot programs, or substutions made by
     *          voice2json while parsing the sentence.
     * @see https://voice2json.org/sentences.html
     */
    struct Slot {
        /**
         * @brief The name of the slot.
         */
        std::string name;
        /**
         * @brief Either the fragment of sentence recognised by voice2json, or a substituted value (both of which
         * are specified by @c sentences.ini )
         */
        std::string value;
    };


    /**
     * @brief A @c SpeechIntentMsg is emitted when intention recognition has successfully recognised an intention
     *        from either a file or a live stream of audio.
     */
    struct SpeechIntentMsg {
        /**
         * @brief Contains the raw translation of the text.
         * @details This is the text field of the json blob from running
         *          @code voice2json transcribe-wav audio.wav @endcode
         */
        std::string text;
        /**
         * @brief The intention of the spoken or recorded command.
         * @details The sentences.ini file defines the valid values for this field.
         */
        std::string intent;
        /**
         * @brief Contains metadata about the substitutions needed to be made in order to match the sentence.
         * @details Each slot contains a name and a value.
         * @see https://voice2json.org/sentences.html
         */
        std::vector<Slot> slots;

        /**
         * A number between 0 and 1.
         * @see https://voice2json.org/formats.html#intents
         * @remark You probably don't want to make any assumptions about this value. Either use empirical evidence
         * or consult voice2json's documentation for a precise interpretation of this value.
         */
        float confidence;

    };

    enum SpeechIntentTranscribeMode {
        TRANSCRIBE_MODE_FILE,
        TRANSCRIBE_MODE_STREAM,
    };

    /**
     * @brief Emits speech and intention recognition messages into nuclear.
     * @details The module activates the microphone after reacting to a enable message. Speech recognition is
     * performed by voice2json subprocesses which use @c arecord under the hood to record audio from the microphone.
     * @see SpeechIntentMessage
     * @see https://voice2json.org
     */
    class SpeechIntent : public NUClear::Reactor {
    private:
        bool output_enabled = true;

        SpawnedProcess voice2json_proc {};

        void init(SpeechIntentTranscribeMode transcribe_mode);
        void recognize_wav(std::string filename);


    public:
        /// @brief Called by the powerplant to build and setup the SpeechIntent reactor.
        explicit SpeechIntent(std::unique_ptr<NUClear::Environment> environment);
        ~SpeechIntent();
    };


}  // namespace module::input

#endif  // MODULE_INPUT_SPEECHINTENT_HPP
