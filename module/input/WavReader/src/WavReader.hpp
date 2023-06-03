#ifndef MODULE_INPUT_WAVREADER_HPP
#define MODULE_INPUT_WAVREADER_HPP

#include <nuclear>
#include <string>
namespace module::input {

    class WavReader : public NUClear::Reactor {
    private:
        /// The configuration variables for this reactor
        struct {
            /// @brief Path of the wav file to convert to text
            std::string wav_path;
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the WavReader reactor.
        explicit WavReader(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Reads the wav file and emits the audio data
        void read_wav();
    };

}  // namespace module::input

#endif  // MODULE_INPUT_WAVREADER_HPP
