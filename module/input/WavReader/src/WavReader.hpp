#ifndef MODULE_INPUT_WAVREADER_HPP
#define MODULE_INPUT_WAVREADER_HPP

#include <nuclear>
#include <string>
namespace module::input {

    class WavReader : public NUClear::Reactor {
    private:
        /// The configuration variables for this reactor
        struct {
            std::string wav_path;
        } config;

    public:
        /// @brief Called by the powerplant to build and setup the WavReader reactor.
        explicit WavReader(std::unique_ptr<NUClear::Environment> environment);
        void readWav();
        //void fessk(FILE*, int, );
    };

}  // namespace module::input

#endif  // MODULE_INPUT_WAVREADER_HPP
