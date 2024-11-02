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
#include "GPT.hpp"

#include "extension/Configuration.hpp"

#include "message/skill/GPT.hpp"
#include "message/skill/Say.hpp"

#include "utility/input/microphone.hpp"
#include "utility/openai/openai.hpp"

namespace module::skill {

    using extension::Configuration;

    using message::skill::GPTAudioRequest;
    using message::skill::GPTChatRequest;
    using message::skill::Say;

    using utility::input::raw_to_mp3;
    using utility::input::record_audio;

    GPT::GPT(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("GPT.yaml").then([this](const Configuration& config) {
            // Use configuration here from file GPT.yaml
            this->log_level    = config["log_level"].as<NUClear::LogLevel>();
            cfg.openai_api_key = config["openai_api_key"].as<std::string>();
            cfg.device_name    = config["device_name"].as<std::string>();
            cfg.pre_prompt     = config["pre_prompt"].as<std::string>();
        });

        on<Startup>().then([this] {
            // Configure the OpenAI API key
            utility::openai::start(cfg.openai_api_key);
        });

        on<Provide<GPTChatRequest>>().then([this](const GPTChatRequest& gpt_request, const RunReason& run_reason) {
            if (run_reason == RunReason::NEW_TASK) {
                // Send request to OpenAI API
                nlohmann::json request = {
                    {"model", "gpt-3.5-turbo"},
                    {"messages",
                     nlohmann::json::array(
                         {{{"role", "user"}, {"content", std::string(cfg.pre_prompt + gpt_request.text)}}})},
                    {"max_tokens", 100},
                    {"temperature", 0}};
                auto chat = utility::openai::chat().create(request);

                std::string response = chat["choices"][0]["message"]["content"].get<std::string>();
                log<NUClear::DEBUG>("Response: ", response);

                if (gpt_request.speak_response) {
                    emit<Task>(std::make_unique<Say>(response));
                }
            }
        });

        on<Provide<GPTAudioRequest>>().then([this](const GPTAudioRequest& gpt_request, const RunReason& run_reason) {
            if (run_reason == RunReason::NEW_TASK) {
                // Record audio for requested time
                log<NUClear::INFO>("Recording audio...");
                record_audio(std::string("audio.raw"), gpt_request.record_time, cfg.device_name);
                log<NUClear::INFO>("Finished recording audio.");

                // Convert audio to mp3
                raw_to_mp3("audio.raw", "audio.mp3");
                log<NUClear::INFO>("Converted audio to mp3 format.");

                // Send request to OpenAI API
                auto audio = utility::openai::audio().transcribe(R"(
                    {
                    "file": "audio.mp3",
                    "model": "whisper-1"
                    }
                )"_json);

                std::string transcription = audio["text"].get<std::string>();
                log<NUClear::DEBUG>("Transcription: ", transcription);

                if (gpt_request.send_to_chat) {
                    emit<Task>(std::make_unique<GPTChatRequest>(transcription, gpt_request.speak_response));
                }
            }
        });
    }

}  // namespace module::skill
