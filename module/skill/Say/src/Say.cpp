/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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

#include "Say.hpp"

#include <fstream>
#include <functional>
#include <iostream>
#include <onnxruntime_cxx_api.h>
#include <optional>
#include <piper-phonemize/phoneme_ids.hpp>
#include <piper-phonemize/phonemize.hpp>
#include <piper-phonemize/tashkeel.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "json.hpp"
#include "piper.hpp"

#include "extension/Configuration.hpp"

#include "message/skill/Say.hpp"

namespace module::skill {

    using json = nlohmann::json;
    using extension::Configuration;
    using SayTask = message::skill::Say;

    Say::Say(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        // Get our configuration from configuration system
        on<Configuration>("Say.yaml").then([this](const Configuration& config) {
            // Store our configuration
            cfg.voice        = config["voice"].as<std::string>();
            cfg.device_name  = config["device_name"].as<std::string>();
            cfg.startup_text = config["startup_text"].as<std::string>();

            // Get espeak data path
            auto espeak_data_path = config["espeak_data_path"].as<std::string>();

            // Get model path
            auto model_path = config["model_path"].as<std::string>();

            // Initialize piper
            // try {
            //     log<INFO>("Initializing piper TTS with voice model:", this->cfg.voice);

            //     // Setup piper configuration
            //     piper_config.eSpeakDataPath = espeak_data_path;

            //     // Load the voice
            //     std::optional<piper::SpeakerId> speakerId;
            //     piper::loadVoice(piper_config,
            //                      model_path + "/" + this->cfg.voice,
            //                      model_path + "/" + this->cfg.voice + ".json",
            //                      voice,
            //                      speakerId,
            //                      false);

            //     // Initialize piper
            //     piper::initialize(piper_config);

            //     // Say startup text if provided
            //     if (!this->cfg.startup_text.empty()) {
            //         log<INFO>("Saying startup text:", this->cfg.startup_text);

            //         // Prepare audio buffer
            //         std::vector<int16_t> audioBuffer;
            //         piper::SynthesisResult result;

            //         // Convert text to audio
            //         piper::textToAudio(piper_config, voice, this->cfg.startup_text, audioBuffer, result, []() {});

            //         // Play audio (implementation depends on your audio playback system)
            //         // playAudio(audioBuffer, this->cfg.device_name);
            //         log<INFO>("Startup text TTS completed in", result.inferSeconds, "seconds");
            //         log<INFO>("Audio length:", result.audioSeconds, "seconds (RTF:", result.realTimeFactor, ")");
            //     }

            // log<INFO>("Piper TTS initialized successfully");
            //     }
            //     catch (const std::exception& e) {
            // log<ERROR>("Failed to initialize piper TTS:", e.what());
            //     }
        });

        // Handle Say requests
        // on<Provide<SayTask>>().then([this](const SayTask& say) {
        //     try {
        //         log<INFO>("Converting text to speech:", say.text);

        //         // Prepare audio buffer
        //         std::vector<int16_t> audioBuffer;
        //         piper::SynthesisResult result;

        //         // Convert text to audio
        //         piper::textToAudio(piper_config, voice, say.text, audioBuffer, result, []() {});

        //         // Play audio (implementation depends on your audio playback system)
        //         // playAudio(audioBuffer, this->cfg.device_name);
        //         log<INFO>("Text to speech completed in", result.inferSeconds, "seconds");
        //         log<INFO>("Audio length:", result.audioSeconds, "seconds (RTF:", result.realTimeFactor, ")");
        //     }
        //     catch (const std::exception& e) {
        //         log<ERROR>("Text to speech failed:", e.what());
        //     }
        // });

        // On shutdown, terminate piper
        // on<Shutdown>().then([this] {
        //     log<INFO>("Shutting down piper TTS");
        //     piper::terminate(piper_config);
        // });
    }
}  // namespace module::skill
