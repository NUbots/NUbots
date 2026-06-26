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

#include "WhistleDetection.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdlib>

#include "extension/Configuration.hpp"

#include "message/input/Whistle.hpp"

#include "utility/nusight/NUhelpers.hpp"

namespace module::input {

    using extension::Configuration;
    using message::input::Whistle;
    using utility::nusight::graph;


    // Radix-2 Cooley-Tukey FFT — in-place, N must be a power of two.
    // Reference: same spectral feature used by NaoDevils' WhistleNet training pipeline.
    static void fft(std::vector<std::complex<float>>& x) {
        const size_t N = x.size();

        // Bit-reversal permutation
        for (size_t i = 1, j = 0; i < N; i++) {
            size_t bit = N >> 1;
            for (; j & bit; bit >>= 1) {
                j ^= bit;
            }
            j ^= bit;
            if (i < j) {
                std::swap(x[i], x[j]);
            }
        }

        // Butterfly stages
        for (size_t len = 2; len <= N; len <<= 1) {
            const float ang = -2.0f * static_cast<float>(M_PI) / static_cast<float>(len);
            const std::complex<float> wlen(std::cos(ang), std::sin(ang));
            for (size_t i = 0; i < N; i += len) {
                std::complex<float> w(1.0f, 0.0f);
                for (size_t j = 0; j < len / 2; j++) {
                    const auto u       = x[i + j];
                    const auto v       = x[i + j + len / 2] * w;
                    x[i + j]           = u + v;
                    x[i + j + len / 2] = u - v;
                    w *= wlen;
                }
            }
        }
    }

    void WhistleDetection::setup_audio() {
        if (pcm_handle != nullptr) {
            snd_pcm_close(pcm_handle);
            pcm_handle = nullptr;
        }

        // The NUBots-built ALSA looks for its config at /usr/local/share/alsa/ which isn't
        // deployed on the robot. Fall back to the system config so device names resolve.
        setenv("ALSA_CONFIG_PATH", "/usr/share/alsa/alsa.conf", 0);

        int err = snd_pcm_open(&pcm_handle, cfg.device.c_str(), SND_PCM_STREAM_CAPTURE, SND_PCM_NONBLOCK);
        if (err < 0) {
            log<WARN>("Cannot open audio device '",
                      cfg.device,
                      "':",
                      snd_strerror(err),
                      "- whistle detection disabled");
            return;
        }

        snd_pcm_hw_params_t* hw_params = nullptr;
        snd_pcm_hw_params_alloca(&hw_params);
        snd_pcm_hw_params_any(pcm_handle, hw_params);
        snd_pcm_hw_params_set_access(pcm_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(pcm_handle, hw_params, SND_PCM_FORMAT_S16_LE);
        snd_pcm_hw_params_set_channels(pcm_handle, hw_params, 1);

        unsigned int actual_rate = cfg.sample_rate;
        snd_pcm_hw_params_set_rate_near(pcm_handle, hw_params, &actual_rate, nullptr);

        // Period = half the FFT window so we accumulate two periods before processing
        snd_pcm_uframes_t period = static_cast<snd_pcm_uframes_t>(cfg.fft_size / 2);
        int dir                  = 0;
        snd_pcm_hw_params_set_period_size_near(pcm_handle, hw_params, &period, &dir);

        if ((err = snd_pcm_hw_params(pcm_handle, hw_params)) < 0) {
            log<ERROR>("Cannot set ALSA hardware parameters:", snd_strerror(err));
            snd_pcm_close(pcm_handle);
            pcm_handle = nullptr;
            return;
        }

        if ((err = snd_pcm_prepare(pcm_handle)) < 0) {
            log<ERROR>("Cannot prepare ALSA interface:", snd_strerror(err));
            snd_pcm_close(pcm_handle);
            pcm_handle = nullptr;
            return;
        }

        sample_buffer.clear();
        consecutive_detections = 0;
        log<INFO>("ALSA capture opened: device=", cfg.device, "rate=", actual_rate, "Hz fft_size=", cfg.fft_size);
    }

    WhistleDetection::WhistleDetection(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("WhistleDetection.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.device               = config["device"].as<std::string>();
            cfg.sample_rate          = config["sample_rate"].as<unsigned int>();
            cfg.fft_size             = config["fft_size"].as<int>();
            cfg.freq_min             = config["freq_min"].as<float>();
            cfg.freq_max             = config["freq_max"].as<float>();
            cfg.band_ratio_threshold = config["band_ratio_threshold"].as<float>();
            cfg.min_band_energy      = config["min_band_energy"].as<float>();
            cfg.confirm_frames       = config["confirm_frames"].as<int>();
            cfg.cooldown_ms          = config["cooldown_ms"].as<int>();

            emit(graph("Whistle/cfg/freq_min", cfg.freq_min));
            emit(graph("Whistle/cfg/freq_max", cfg.freq_max));
            emit(graph("Whistle/cfg/band_ratio_threshold", cfg.band_ratio_threshold));
            emit(graph("Whistle/cfg/min_band_energy", cfg.min_band_energy));
            emit(graph("Whistle/cfg/confirm_frames", static_cast<float>(cfg.confirm_frames)));

            setup_audio();
        });

        // Poll for audio every 10 ms. Non-blocking reads accumulate into sample_buffer;
        // once a full FFT window is ready, process_frame is called with 50% overlap.
        audio_handle = on<Every<10, std::chrono::milliseconds>>().then([this] {
            if (pcm_handle == nullptr) {
                return;
            }

            // Read up to half a window of samples at a time
            const int read_size = cfg.fft_size / 2;
            std::vector<int16_t> raw(read_size);
            const snd_pcm_sframes_t n = snd_pcm_readi(pcm_handle, raw.data(), read_size);

            if (n == -EAGAIN) {
                return;  // no data available yet
            }

            if (n < 0) {
                if (snd_pcm_recover(pcm_handle, static_cast<int>(n), 0) < 0) {
                    log<WARN>("ALSA read error (unrecoverable):", snd_strerror(static_cast<int>(n)));
                }
                return;
            }

            // Convert S16_LE to normalised float [-1, 1]
            for (snd_pcm_sframes_t i = 0; i < n; i++) {
                sample_buffer.push_back(static_cast<float>(raw[i]) / 32768.0f);
            }

            // Process all available complete windows (50% overlap → hop = fft_size/2)
            while (static_cast<int>(sample_buffer.size()) >= cfg.fft_size) {
                process_frame(std::vector<float>(sample_buffer.begin(), sample_buffer.begin() + cfg.fft_size));
                sample_buffer.erase(sample_buffer.begin(), sample_buffer.begin() + cfg.fft_size / 2);
            }
        });
    }

    void WhistleDetection::process_frame(const std::vector<float>& samples) {
        const int N = cfg.fft_size;

        // Apply Hanning window and load into complex vector
        std::vector<std::complex<float>> x(N);
        for (int i = 0; i < N; i++) {
            const float hann = 0.5f * (1.0f - std::cos(2.0f * static_cast<float>(M_PI) * i / (N - 1)));
            x[i]             = {samples[i] * hann, 0.0f};
        }

        fft(x);

        // Power spectrum over the one-sided range [1, N/2] (skip DC at bin 0)
        const int bins = N / 2 + 1;

        const float bin_hz = static_cast<float>(cfg.sample_rate) / static_cast<float>(N);
        const int bin_min  = std::max(1, static_cast<int>(cfg.freq_min / bin_hz));
        const int bin_max  = std::min(bins, static_cast<int>(std::ceil(cfg.freq_max / bin_hz)));

        float total_energy = 0.0f;
        float band_energy  = 0.0f;
        for (int i = 1; i < bins; i++) {
            const float power = std::norm(x[i]);  // |x[i]|^2
            total_energy += power;
            if (i >= bin_min && i < bin_max) {
                band_energy += power;
            }
        }

        emit(graph("Whistle/energy/total", total_energy));
        emit(graph("Whistle/energy/band", band_energy));

        // Skip frames that are essentially silence
        if (total_energy < 1e-10f) {
            consecutive_detections = 0;
            return;
        }

        const float band_ratio     = band_energy / total_energy;
        const bool above_threshold = (band_ratio > cfg.band_ratio_threshold) && (band_energy > cfg.min_band_energy);

        emit(graph("Whistle/band_ratio", band_ratio));
        emit(graph("Whistle/band_ratio_threshold", cfg.band_ratio_threshold));

        if (above_threshold) {
            consecutive_detections++;
            log<DEBUG>("band_ratio=",
                       band_ratio,
                       "bins",
                       bin_min,
                       "/",
                       bin_max,
                       "consecutive=",
                       consecutive_detections);
        }
        else {
            consecutive_detections = 0;
        }

        emit(graph("Whistle/consecutive_detections", static_cast<float>(consecutive_detections)));

        if (consecutive_detections >= cfg.confirm_frames) {
            const auto now        = NUClear::clock::now();
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_detection).count();

            if (elapsed_ms >= cfg.cooldown_ms) {
                const float confidence = std::min(1.0f, band_ratio / cfg.band_ratio_threshold);

                auto msg        = std::make_unique<Whistle>();
                msg->time       = now;
                msg->confidence = confidence;
                emit(std::move(msg));

                emit(graph("Whistle/confidence", confidence));
                emit(graph("Whistle/detected", 1.0f));

                log<INFO>("Whistle detected! confidence=", confidence, "band_ratio=", band_ratio);

                last_detection         = now;
                consecutive_detections = 0;
            }
        }
        else {
            emit(graph("Whistle/detected", 0.0f));
        }
    }

    WhistleDetection::~WhistleDetection() {
        if (pcm_handle != nullptr) {
            snd_pcm_close(pcm_handle);
            pcm_handle = nullptr;
        }
    }

}  // namespace module::input
