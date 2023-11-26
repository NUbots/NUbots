#include "GPT.hpp"

#include "extension/Configuration.hpp"

#include "message/skill/GPT.hpp"
#include "message/skill/Say.hpp"

#include "utility/openai/openai.hpp"

namespace module::skill {

    using extension::Configuration;
    using message::skill::GPTRequest;
    using message::skill::Say;

    GPT::GPT(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("GPT.yaml").then([this](const Configuration& config) {
            // Use configuration here from file GPT.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.model       = config["model"].as<std::string>();
            cfg.max_tokens  = config["max_tokens"].as<int>();
            cfg.temperature = config["temperature"].as<double>();
        });

        on<Configuration>("OPENAI_API_KEY.yaml").then([this](const Configuration& config) {
            cfg.openai_api_key = config["openai_api_key"].as<std::string>();
        });

        on<Startup>().then([this] {
            // Configure the OpenAI API key
            utility::openai::start(cfg.openai_api_key);
        });

        on<Provide<GPTRequest>>().then([this](const GPTRequest& gpt_request, const RunInfo& info) {
            if (info.run_reason == RunInfo::NEW_TASK) {
                // Send request to OpenAI API
                nlohmann::json request = {
                    {"model", cfg.model},
                    {"messages", nlohmann::json::array({{{"role", "user"}, {"content", gpt_request.text}}})},
                    {"max_tokens", cfg.max_tokens},
                    {"temperature", cfg.temperature}};
                auto chat = utility::openai::chat().create(request);

                std::string response = chat["choices"][0]["message"]["content"].get<std::string>();
                log<NUClear::DEBUG>("Response: {}", response);

                if (gpt_request.speak_response) {
                    emit<Task>(std::make_unique<Say>(response));
                }
            }
        });
    }

}  // namespace module::skill
