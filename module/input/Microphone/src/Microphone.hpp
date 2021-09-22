#ifndef MODULE_INPUT_MICROPHONE_HPP
#define MODULE_INPUT_MICROPHONE_HPP

#include <nuclear>

namespace module::input {

enum MicMsgType {
    MIC_MSG_ENABLE,
    MIC_MSG_DISABLE,
    MIC_MSG_TEST_AUDIO,
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


class SpeechIntentMessage {
public:
    std::string text;
    std::string intent;
    std::vector<Slot> slots;
    float confidence;
    
    void add_slot(std::string name, std::string value) {
        Slot slot = {name, value};
        slots.push_back(slot);
    }
};

class Microphone : public NUClear::Reactor {
private:
    /// The configuration variables for this reactor
    struct {
       bool debug_mode = false;
    } config;
    bool enabled = true;
    //MicProcHandles handles;
    
    SpawnedProcess voice2json_proc;

public:
    /// @brief Called by the powerplant to build and setup the Microphone reactor.
    explicit Microphone(std::unique_ptr<NUClear::Environment> environment);
    ~Microphone();
};



}  // namespace module::input

#endif  // MODULE_INPUT_MICROPHONE_HPP
