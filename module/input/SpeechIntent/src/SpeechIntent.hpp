#ifndef MODULE_INPUT_SPEECHINTENT_HPP
#define MODULE_INPUT_SPEECHINTENT_HPP

#include <nuclear>

namespace module::input {

    enum MicMsgType {
        MIC_MSG_ENABLE,
        MIC_MSG_DISABLE,
        MIC_MSG_RECOGNIZE_AUDIO_FROM_FILE,
    };

    struct MicControlMsg {
        MicMsgType type;
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


    struct SpeechIntentMessage {
        std::string text;
        std::string intent;
        std::vector<Slot> slots;
        float confidence;

        void add_slot(std::string name, std::string value) {
            Slot slot = {name, value};
            slots.push_back(slot);
        }
    };

    class SpeechIntent : public NUClear::Reactor {
    private:
        /// The configuration variables for this reactor
        struct {
            bool debug_mode = false;
        } config;
        bool enabled = true;
        // MicProcHandles handles;

        SpawnedProcess voice2json_proc;

    public:
        /// @brief Called by the powerplant to build and setup the SpeechIntent reactor.
        explicit SpeechIntent(std::unique_ptr<NUClear::Environment> environment);
        ~SpeechIntent();
    };


}  // namespace module::input

#endif  // MODULE_INPUT_SPEECHINTENT_HPP
