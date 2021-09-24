#ifndef MODULE_INPUT_SPEECHINTENT_HPP
#define MODULE_INPUT_SPEECHINTENT_HPP

#include <spawn.h>

#include <nuclear>

namespace module::input {

    struct SpeechInputSetOutputMsg {
        bool enabled;
    };
    
    struct SpeechInputRecognizeWavFile {
        std::string filename;
    };

    struct SpawnedProcess {
        int stdout;
        int stderr;
        int stdin;
        pid_t pid;

        std::string process_name;
    };

    struct Slot {
        std::string name;
        std::string value;
    };


    struct SpeechIntentMsg {
        std::string text;
        std::string intent;
        std::vector<Slot> slots;
        float confidence;

        void add_slot(std::string name, std::string value) {
            Slot slot = {name, value};
            slots.push_back(slot);
        }
    };
    
    struct ProcessSpawnResources {
        int stdout_pipe[2]                 = {};
        int stderr_pipe[2]                 = {};
        int stdin_pipe[2]                  = {};
        posix_spawn_file_actions_t actions = {};
        bool actions_inited                = false;
        std::string env_path;
        
        ~ProcessSpawnResources() {
            if (stdout_pipe[1])
                close(stdout_pipe[1]);
            if (stderr_pipe[1])
                close(stderr_pipe[1]);
            if (stdin_pipe[0])
                close(stdin_pipe[0]);
            if (actions_inited) {
                posix_spawn_file_actions_destroy(&actions);
            }
        }
    };
    
    enum SpeechIntentTranscribeMode {
        TRANSCRIBE_MODE_FILE,
        TRANSCRIBE_MODE_STREAM,
    };

    class SpeechIntent : public NUClear::Reactor {
    private:
        /// The configuration variables for this reactor
        struct {
            bool debug_mode = false;
            SpeechIntentTranscribeMode transcribe_mode = TRANSCRIBE_MODE_STREAM;
        } config;
        bool output_enabled = true;
        // MicProcHandles handles;

        SpawnedProcess voice2json_proc = {};
        
        void init();
        void recognize_wav(std::string filename);


    public:
        /// @brief Called by the powerplant to build and setup the SpeechIntent reactor.
        explicit SpeechIntent(std::unique_ptr<NUClear::Environment> environment);
        ~SpeechIntent();
    };


}  // namespace module::input

#endif  // MODULE_INPUT_SPEECHINTENT_HPP
