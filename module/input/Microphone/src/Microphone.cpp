#include "Microphone.hpp"

#include "PortAudio.h"

#include "extension/Configuration.hpp"

namespace module::input {

    using extension::Configuration;

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
                                         PaStreamCallback,             /* this is your callback function */
                                         &data);                       /*This is a pointer that will be passed to
                                                                                 your callback*/
            check_error(error);

            // Start the microphone stream
            error = Pa_StartStream(stream);
            check_error(error);
        });

        on<Shutdown>().then([]() {
            // Abort to stop the callback processing audio data
            Pa_AbortStream();
            // Close the stream to free up resources
            error = Pa_CloseStream(stream);
            check_error(error);
            // Terminate
            error = Pa_Terminate();
            check_error(error);
        });
    }

    // PortAudio callback function
    int PaStreamCallback(const void* input,
                         void* output,
                         unsigned long frameCount,
                         const PaStreamCallbackTimeInfo* timeInfo,
                         PaStreamCallbackFlags statusFlags,
                         void* userData) {

        /* copied from example, to look through
        paTestData* data   = (paTestData*) userData;
        const SAMPLE* rptr = (const SAMPLE*) inputBuffer;
        SAMPLE* wptr       = &data->recordedSamples[data->frameIndex * NUM_CHANNELS];
        long framesToCalc;
        long i;
        int finished;
        unsigned long framesLeft = data->maxFrameIndex - data->frameIndex;

        (void) outputBuffer; // Prevent unused variable warnings.
        (void) timeInfo;
        (void) statusFlags;
        (void) userData;

        if (framesLeft < framesPerBuffer) {
            framesToCalc = framesLeft;
            finished     = paComplete;
        }
        else {
            framesToCalc = framesPerBuffer;
            finished     = paContinue;
        }

        if (inputBuffer == NULL) {
            for (i = 0; i < framesToCalc; i++) {
                *wptr++ = SAMPLE_SILENCE; // left
                if (NUM_CHANNELS == 2)
                    *wptr++ = SAMPLE_SILENCE; // right
            }
        }
        else {
            for (i = 0; i < framesToCalc; i++) {
                *wptr++ = *rptr++; // left
                if (NUM_CHANNELS == 2)
                    *wptr++ = *rptr++; // right
            }
        }
        data->frameIndex += framesToCalc;
        return finished;*/
    }

    void Microphone::check_error(PaError error) {
        if (error != paNoError) {
            throw std::runtime("Error: ", Pa_GetErrorTest(err));
        }
    }
}  // namespace module::input
