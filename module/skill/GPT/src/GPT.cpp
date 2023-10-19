#include "GPT.hpp"

#include "extension/Configuration.hpp"

#include "message/skill/GPT.hpp"

#include "utility/openai/openai.hpp"

namespace module::skill {

    using extension::Configuration;
    using message::skill::GPTRequest;
    using message::skill::GPTResponse;

    GPT::GPT(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("GPT.yaml").then([this](const Configuration& config) {
            // Use configuration here from file GPT.yaml
            this->log_level    = config["log_level"].as<NUClear::LogLevel>();
            cfg.openai_api_key = config["openai_api_key"].as<std::string>();
        });

        on<Startup>().then([this] {
            // Configure the OpenAI API key
            utility::openai::start(cfg.openai_api_key);
        });

        on<Provide<GPTRequest>>().then([this](const GPTRequest& chatgpt_request, const RunInfo& info) {
            if (info.run_reason == RunInfo::NEW_TASK) {
                // Send request to OpenAI API
                nlohmann::json request = {
                    {"model", "gpt-3.5-turbo"},
                    {"messages", nlohmann::json::array({{{"role", "user"}, {"content", chatgpt_request.text}}})},
                    {"max_tokens", 100},
                    {"temperature", 0}};
                auto chat = utility::openai::chat().create(request);

                std::string response = chat["choices"][0]["message"]["content"].get<std::string>();
                log<NUClear::DEBUG>("Response: {}", response);

                emit(std::make_unique<GPTResponse>(response));
            }
        });
    }

}  // namespace module::skill
