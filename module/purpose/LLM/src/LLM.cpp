#include "LLM.hpp"

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/planning/KickTo.hpp"
#include "message/planning/LookAround.hpp"
#include "message/purpose/LLM.hpp"
#include "message/strategy/AlignBallToGoal.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/FindFeature.hpp"
#include "message/strategy/KickToGoal.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/Ready.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

#include "utility/openai/openai.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::purpose {

    using extension::Configuration;

    using message::planning::KickTo;
    using message::planning::LookAround;
    using message::strategy::AlignBallToGoal;
    using message::strategy::FallRecovery;
    using message::strategy::FindBall;
    using message::strategy::KickToGoal;
    using message::strategy::LookAtBall;
    using message::strategy::Ready;
    using message::strategy::StandStill;
    using message::strategy::WalkToBall;
    using message::strategy::WalkToFieldPosition;
    using LLMTask = message::purpose::LLM;
    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;

    using utility::support::Expression;

    LLM::LLM(std::unique_ptr<NUClear::Environment> environment) : BehaviourReactor(std::move(environment)) {

        on<Configuration>("LLM.yaml").then([this](const Configuration& config) {
            // Use configuration here from file LLM.yaml
            this->log_level                     = config["log_level"].as<NUClear::LogLevel>();
            cfg.find_ball_priority              = config["tasks"]["find_ball_priority"].as<int>();
            cfg.look_at_ball_priority           = config["tasks"]["look_at_ball_priority"].as<int>();
            cfg.walk_to_ball_priority           = config["tasks"]["walk_to_ball_priority"].as<int>();
            cfg.align_ball_to_goal_priority     = config["tasks"]["align_ball_to_goal_priority"].as<int>();
            cfg.kick_to_goal_priority           = config["tasks"]["kick_to_goal_priority"].as<int>();
            cfg.walk_to_field_position_priority = config["tasks"]["walk_to_field_position_priority"].as<int>();
            cfg.kick_to_priority                = config["tasks"]["kick_to_priority"].as<int>();
            cfg.look_around_priority            = config["tasks"]["look_around_priority"].as<int>();
            cfg.stand_still_priority            = config["tasks"]["stand_still_priority"].as<int>();
            cfg.walk_to_field_position_position = config["walk_to_field_position_position"].as<Expression>();
            cfg.openai_api_key                  = config["openai_api_key"].as<std::string>();
            cfg.user_request                    = config["user_request"].as<std::string>();
        });

        on<Startup>().then([this] {
            utility::openai::start(cfg.openai_api_key);
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED));
            // This emit starts the tree to achieve the purpose
            emit<Task>(std::make_unique<LLMTask>());
            // The robot should always try to recover from falling, if applicable, regardless of purpose
            emit<Task>(std::make_unique<FallRecovery>(), 1);
        });

        // TODO: Add "world model" (SensorFilter, Localisation, etc.) using With, and then use it in the prompt, maybe
        // have this update at a certain rate such that the request is received?
        on<Provide<LLMTask>>().then([this](const LLMTask& llm_task) {
            // TODO: Build prompt for LLM in a way which incorporates the current world state and provides a sensible
            // and parsable response, something along the lines of:
            std::string prompt = "Given desired user request: " + cfg.user_request + " and current world state: Ball was last seen 0.1 seconds ago 0.1m away from you. You have the ability to WalkToBall, Kick, LookAround, StandStill. What tasks should you currently do? \n Provide your response as a list of with format: Task: <task> Priority: <priority> \n where <task> is one of the aforementioned tasks, and <priority> is an integer above 0, a higher number means a higher priority and will be take control over a lower priority tasks servos. \n";

            // Send request to OpenAI API
            nlohmann::json request = {{"model", "text-davinci-003"},
                                      {"prompt", prompt},
                                      {"max_tokens", 100},
                                      {"temperature", 0}};
            auto completion        = utility::openai::completion().create(request);

            // TODO: Parse response, and get the tasks to be emitted
            auto response = completion["choices"][0]["text"].get<std::string>();
            log<NUClear::INFO>("Response is:\n", response);

            // Parse the response to extract tasks and priorities
            std::string task;
            int priority;
            std::regex r(R"(Task: (\w+) Priority: (\d+))");
            std::sregex_iterator it(response.begin(), response.end(), r);
            std::sregex_iterator reg_end;

            for (; it != reg_end; ++it) {
                task     = (*it)[1].str();
                priority = std::stoi((*it)[2].str());

                if (task == "FindBall") {
                    emit<Task>(std::make_unique<FindBall>(), priority);
                }
                else if (task == "LookAtBall") {
                    emit<Task>(std::make_unique<LookAtBall>(), priority);
                }
                else if (task == "WalkToBall") {
                    emit<Task>(std::make_unique<WalkToBall>(), priority);
                }
                else if (task == "AlignBallToGoal") {
                    emit<Task>(std::make_unique<AlignBallToGoal>(), priority);
                }
                else if (task == "KickToGoal") {
                    emit<Task>(std::make_unique<KickToGoal>(), priority);
                }
                else if (task == "WalkToFieldPosition") {
                    emit<Task>(
                        std::make_unique<WalkToFieldPosition>(Eigen::Vector3f(cfg.walk_to_field_position_position.x(),
                                                                              cfg.walk_to_field_position_position.y(),
                                                                              0),
                                                              cfg.walk_to_field_position_position.z()),
                        priority);
                }
                else if (task == "Kick") {
                    emit<Task>(std::make_unique<KickTo>(), priority);
                }
                else if (task == "LookAround") {
                    emit<Task>(std::make_unique<LookAround>(), priority);
                }
                else if (task == "StandStill") {
                    emit<Task>(std::make_unique<StandStill>(), priority);
                }
                else {
                    log<NUClear::ERROR>("Unexpected task from LLM response: ", task);
                }
            }


            // Profit.
        });
    }

}  // namespace module::purpose
