#include "Microphone.hpp"

#include <iostream>
#include <stdexcept>

#include "portaudio.h"

#include "extension/Configuration.hpp"

namespace module::input {

    using extension::Configuration;

    // PortAudio callback function
    static int microphone_callback(const void* input,
                                   void* output,
                                   unsigned long frameCount,
                                   const PaStreamCallbackTimeInfo* timeInfo,
                                   PaStreamCallbackFlags statusFlags,
                                   void* userData) {

        float* microphone_input    = (float*) input;
        std::vector<float> m_input = {};

        // log<NUClear::DEBUG>(microphone_input);

        for (long unsigned int i = 0; i < frameCount; i++) {
            m_input.push_back(microphone_input[i]);
            std::cout << microphone_input[i] << std::endl;
        }


        return 0;
    }

    Microphone::Microphone(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {

        on<Configuration>("Microphone.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file Microphone.yaml
            this->log_level    = cfg["log_level"].as<NUClear::LogLevel>();
            config.sample_rate = cfg["sample_rate"].as<int>();

            // INITIALISE AUDIO
            error = Pa_Initialize();
            check_error(error);

            // Set up callback stuff
            error = Pa_OpenDefaultStream(&stream,
                                         1,         /* one input channel */
                                         0,         /* no output */
                                         paFloat32, /* 32 bit floating point output */
                                         config.sample_rate,
                                         paFramesPerBufferUnspecified, /* frames per buffer, i.e. the number
                                                     of sample frames that PortAudio will
                                                     request from the callback. Many apps
                                                     may want to use
                                                     paFramesPerBufferUnspecified, which
                                                     tells PortAudio to pick the best,
                                                     possibly changing, buffer size.*/
                                         microphone_callback,          /* this is your callback function */
                                         nullptr);                     /*This is a pointer that will be passed to
                                                                               your callback*/
            check_error(error);

            // Start the microphone stream
            error = Pa_StartStream(stream);
            check_error(error);
        });

        on<Shutdown>().then([this]() {
            // Abort to stop the callback processing audio data
            Pa_AbortStream(stream);
            // Close the stream to free up resources
            error = Pa_CloseStream(stream);
            check_error(error);
            // Terminate
            error = Pa_Terminate();
            check_error(error);
        });
    }


    void Microphone::check_error(PaError err) {
        if (err != paNoError) {
            throw std::runtime_error(fmt::format("Error: {}", Pa_GetErrorText(err)));
        }
    }
}  // namespace module::input
