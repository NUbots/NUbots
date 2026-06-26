/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef MODULE_INPUT_WHISTLEDETECTION_HPP
#define MODULE_INPUT_WHISTLEDETECTION_HPP

#include <alsa/asoundlib.h>
#include <nuclear>
#include <vector>

namespace module::input {

    class WhistleDetection : public NUClear::Reactor {
    private:
        struct Config {
            /// ALSA device name
            std::string device{"default"};
            /// Capture sample rate in Hz
            unsigned int sample_rate{16000};
            /// FFT window size (must be a power of two)
            int fft_size{1024};
            /// Lower bound of the whistle frequency band in Hz
            float freq_min{2000.0f};
            /// Upper bound of the whistle frequency band in Hz
            float freq_max{4500.0f};
            /// Fraction of total spectral power that must lie in the whistle band
            float band_ratio_threshold{0.40f};
            /// Minimum absolute in-band power (filters near-silence)
            float min_band_energy{1e-6f};
            /// Number of consecutive FFT frames above threshold required to confirm a whistle
            int confirm_frames{3};
            /// Minimum time between consecutive whistle events in milliseconds
            int cooldown_ms{1500};
            /// Milliseconds to suppress detections after startup (avoids triggering on boot beep)
            int startup_delay{1000};
            /// Whether to use whistle detection or not
            bool use_whistle_detection{true};
        } cfg;

        /// ALSA PCM capture handle
        snd_pcm_t* pcm_handle{nullptr};

        /// Rolling audio sample buffer used to accumulate frames for FFT
        std::vector<float> sample_buffer{};

        /// Number of consecutive FFT frames that have exceeded the detection threshold
        int consecutive_detections{0};

        /// Time of the last emitted Whistle event (used for cooldown)
        NUClear::clock::time_point last_detection{};

        /// Time the module was configured (used to suppress detections during startup)
        NUClear::clock::time_point start_time{};

        /// Handle for the periodic audio-read reaction (allows disabling during reconfiguration)
        ReactionHandle audio_handle{};

        /// Opens (or re-opens) the ALSA capture device with the current configuration
        void setup_audio();

        /// Runs the FFT-based whistle detection pipeline on one window of samples
        void process_frame(const std::vector<float>& samples);

    public:
        /// @brief Called by the powerplant to build and setup the WhistleDetection reactor.
        explicit WhistleDetection(std::unique_ptr<NUClear::Environment> environment);

        ~WhistleDetection();
    };

}  // namespace module::input

#endif  // MODULE_INPUT_WHISTLEDETECTION_HPP
