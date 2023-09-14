#include "Chat.hpp"

#include <sstream>
#include <string>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/skill/Speak.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/StandStill.hpp"

#include "utility/openai/openai.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::purpose {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::skill::Speak;
    using message::strategy::FallRecovery;
    using message::strategy::StandStill;

    Chat::Chat(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("Chat.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Chat.yaml
            this->log_level    = config["log_level"].as<NUClear::LogLevel>();
            cfg.openai_api_key = config["openai_api_key"].as<std::string>();
            cfg.prompt         = config["prompt"].as<std::string>();
        });

        on<Startup>().then([this] {
            // Stand and getup skills
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED));
            emit<Task>(std::make_unique<StandStill>(), 0);  // if not doing anything else, just stand
            // This emit starts the tree to achieve the purpose
            // The robot should always try to recover from falling, if applicable, regardless of purpose
            emit<Task>(std::make_unique<FallRecovery>(), 2);


            utility::openai::start(cfg.openai_api_key);

            // Send request to OpenAI API
            nlohmann::json request = {
                {"model", "gpt-3.5-turbo"},
                {"messages", nlohmann::json::array({{{"role", "user"}, {"content", cfg.prompt}}})},
                {"max_tokens", 100},
                {"temperature", 0}};
            auto chat = utility::openai::chat().create(request);

            std::string response = chat["choices"][0]["message"]["content"].get<std::string>();

            log<NUClear::INFO>("Prompt: ", cfg.prompt);
            log<NUClear::INFO>("Response: ", response);

            emit<Task>(std::make_unique<Speak>(response));
        });
    }

}  // namespace module::purpose
