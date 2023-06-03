#ifndef MODULE_SPEECHRECOGNITION_HPP
#define MODULE_SPEECHRECOGNITION_HPP

#include <nuclear>

namespace module::sound {

    class SpeechRecognition : public NUClear::Reactor {
    private:
        /// The configuration variables for this reactor
        struct {
            std::string model_path = "";
        } config;

        VoskModel* model;
        VoskRecognizer* recognizer;

    public:
        /// @brief Called by the powerplant to build and setup the SpeechRecognition reactor.
        explicit SpeechRecognition(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::sound

#endif  // MODULE_SPEECHRECOGNITION_HPP
