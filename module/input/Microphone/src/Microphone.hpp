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
    uint32_t data;
};


struct MicProcHandles {
    int stdout;
    int stderr;
    int stdin;
};

struct MicSpeechIntent {
    
    
};

struct Voice2jsonParsedIntent {
    char *text;
    char *intent;
    float confidence;
};

class Microphone : public NUClear::Reactor {
private:
    /// The configuration variables for this reactor
    struct {
    } config;
    bool enabled = true;
    MicProcHandles handles;

public:
    /// @brief Called by the powerplant to build and setup the Microphone reactor.
    explicit Microphone(std::unique_ptr<NUClear::Environment> environment);
    ~Microphone();
};



}  // namespace module::input

#endif  // MODULE_INPUT_MICROPHONE_HPP
