#include "LLM.hpp"

#include <sstream>
#include <string>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/planning/KickTo.hpp"
#include "message/planning/LookAround.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/purpose/LLM.hpp"
#include "message/skill/Kick.hpp"
#include "message/skill/Look.hpp"
#include "message/skill/Walk.hpp"
#include "message/strategy/AlignBallToGoal.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/FindFeature.hpp"
#include "message/strategy/KickToGoal.hpp"
#include "message/strategy/LookAtFeature.hpp"
#include "message/strategy/Ready.hpp"
#include "message/strategy/StandStill.hpp"
#include "message/strategy/WalkToBall.hpp"
#include "message/strategy/WalkToFieldPosition.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/openai/openai.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::purpose {

    using extension::Configuration;

    using message::behaviour::state::Stability;
    using message::behaviour::state::WalkState;
    using message::input::Sensors;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::planning::KickTo;
    using message::planning::LookAround;
    using message::planning::TurnOnSpot;
    using message::purpose::LLMPlanner;
    using message::purpose::LLMStrategy;
    using message::skill::Kick;
    using message::skill::Look;
    using message::skill::Walk;
    using message::strategy::AlignBallToGoal;
    using message::strategy::FallRecovery;
    using message::strategy::FindBall;
    using message::strategy::KickToGoal;
    using message::strategy::LookAtBall;
    using message::strategy::Ready;
    using message::strategy::StandStill;
    using message::strategy::WalkToBall;
    using message::strategy::WalkToFieldPosition;

    using utility::input::LimbID;
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

            cfg.main_prompt = config["main_prompt"].as<std::string>();

            cfg.planner      = config["planner"].as<bool>();
            cfg.debug_no_api = config["debug_no_api"].as<bool>();
        });

        on<Startup>().then([this] {
            utility::openai::start(cfg.openai_api_key);
            emit(std::make_unique<Stability>(Stability::UNKNOWN));
            emit(std::make_unique<WalkState>(WalkState::State::STOPPED));
            emit<Task>(std::make_unique<StandStill>(), 0);  // if not doing anything else, just stand
            // This emit starts the tree to achieve the purpose
            // The robot should always try to recover from falling, if applicable, regardless of purpose
            emit<Task>(std::make_unique<FallRecovery>(), 2);

            if (cfg.planner) {

                log<NUClear::INFO>("Running LLM as planner.");
                emit<Task>(std::make_unique<LLMPlanner>(), 1);
            }
            else {
                log<NUClear::INFO>("Running LLM as strategy.");
                emit<Task>(std::make_unique<LLMStrategy>(), 1);
            }
        });


        on<Provide<LLMPlanner>,
           Every<PROMPT_FREQ, std::chrono::seconds>,
           Optional<With<Ball>>,
           Optional<With<Field>>,
           With<Sensors>,
           Single>()
            .then([this](const std::shared_ptr<const Ball>& ball,
                         const std::shared_ptr<const Field>& field,
                         const Sensors& sensors) {
                std::string prompt = cfg.main_prompt;
                prompt += "\nState information:";

                Eigen::Isometry3d Htw(sensors.Htw);

                if (field) {
                    Eigen::Isometry3d Htf = Htw * field->Hfw.inverse();
                    prompt += "\nTorso to field position of the robot is (";
                    prompt += std::to_string(Htf.translation().x());
                    prompt += ",";
                    prompt += std::to_string(Htf.translation().y());
                    prompt += ",";
                    prompt += std::to_string(Htf.translation().z());
                    prompt += ").";
                }
                else {
                    prompt += "\nField: unknown.";
                }

                if (ball) {
                    double time_since_ball_seen = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                      NUClear::clock::now() - ball->time_of_measurement)
                                                      .count()
                                                  / 1000.0;
                    Eigen::Vector3d rBTt = Htw * ball->rBWw;

                    prompt += "\nBall last seen ";
                    prompt += std::to_string(time_since_ball_seen);
                    prompt += " seconds ago at position (";
                    prompt += std::to_string(rBTt.x());
                    prompt += ",";
                    prompt += std::to_string(rBTt.y());
                    prompt += ",";
                    prompt += std::to_string(rBTt.z());
                    prompt += ") from torso to ball.";
                }
                else {
                    prompt += "\nBall: unknown.";
                }

                prompt += "\nRespond with only the function calls, nothing else.";

                log<NUClear::INFO>("Prompt is:\n", prompt);

                std::string response = "";

                // Testing without OpenAI API calls
                if (cfg.debug_no_api) {
                    response = "Look(Eigen::Vector3d(1.6,-0.001,0.23))\nWalk(Eigen::Vector3d(1.0,0.0,0.0))";
                }
                else {
                    // Send request to OpenAI API
                    nlohmann::json request = {
                        {"model", "gpt-3.5-turbo"},
                        {"messages", nlohmann::json::array({{{"role", "user"}, {"content", prompt}}})},
                        {"max_tokens", 200},
                        {"temperature", 0}};
                    auto chat = utility::openai::chat().create(request);
                    response  = chat["choices"][0]["message"]["content"].get<std::string>();
                }

                log<NUClear::INFO>("Response is:\n", response);

                // Parse the response to extract tasks and priorities
                // Parse the Look task
                size_t look_pos = response.find("Look");
                try {
                    if (look_pos != std::string::npos) {
                        // Skip to the Eigen vector
                        look_pos += 20;
                        // Find where the arguments are
                        size_t first_bracket  = response.find("(", look_pos);
                        size_t second_bracket = response.find(")", first_bracket);

                        // Get the arguments
                        // Create a vector from split view
                        std::vector<double> args{};
                        std::stringstream ss(response.substr(first_bracket + 1, second_bracket - first_bracket - 1));

                        while (ss.good()) {
                            std::string substr;
                            std::getline(ss, substr, ',');
                            args.push_back(std::stod(substr));
                        }

                        emit<Task>(std::make_unique<Look>(Eigen::Vector3d(args[0], args[1], args[2])), 0);
                    }
                }
                catch (std::exception& e) {
                    log<NUClear::ERROR>("Response for Look not formatted correctly.");
                }

                // Parse the walk task
                size_t walk_pos = response.find("Walk");
                try {
                    if (walk_pos != std::string::npos) {
                        // Skip to the Eigen vector
                        walk_pos += 20;
                        // Find where the arguments are
                        size_t first_bracket  = response.find("(", walk_pos);
                        size_t second_bracket = response.find(")", first_bracket);

                        // Get the arguments
                        // Create a vector from split view
                        std::vector<double> args{};
                        std::stringstream ss(response.substr(first_bracket + 1, second_bracket - first_bracket - 1));

                        while (ss.good()) {
                            std::string substr;
                            std::getline(ss, substr, ',');
                            args.push_back(std::stod(substr));
                        }

                        emit<Task>(std::make_unique<Walk>(Eigen::Vector3d(args[0], args[1], args[2])), 0);
                    }
                }
                catch (std::exception& e) {
                    log<NUClear::ERROR>("Response for Walk not formatted correctly.");
                }

                // Parse the kick task
                size_t kick_pos = response.find("Kick");
                try {
                    if (kick_pos != std::string::npos) {
                        // Skip to the Eigen vector
                        kick_pos += 20;
                        // Find where the arguments are
                        size_t first_bracket  = response.find("(", kick_pos);
                        size_t second_bracket = response.find(")", first_bracket);
                        // Get the kick direction
                        auto arg        = response.substr(first_bracket + 1, second_bracket - first_bracket - 1);
                        LimbID kick_leg = (arg == "true") || (arg == "True") || (arg == "TRUE") || (arg == "1")
                                              ? LimbID::LEFT_LEG
                                              : LimbID::RIGHT_LEG;
                        emit<Task>(std::make_unique<Kick>(kick_leg), 0);
                    }
                }
                catch (std::exception& e) {
                    log<NUClear::ERROR>("Response for Kick not formatted correctly.");
                }
            });

        // TODO: Add "world model" (SensorFilter, Localisation, etc.) using With, and then use it in the prompt,
        // maybe have this update at a certain rate such that the request is received?
        on<Provide<LLMStrategy>,
           Every<PROMPT_FREQ, std::chrono::seconds>,
           Optional<With<Ball>>,
           Optional<With<Field>>,
           With<Sensors>,
           Single>()
            .then([this](const std::shared_ptr<const Ball>& ball,
                         const std::shared_ptr<const Field>& field,
                         const Sensors& sensors) {
                std::string time_since_ball_seen = "unknown";
                std::string distance_to_ball     = "unknown";
                std::string ball_is_visible      = "is not currently visible";
                if (ball) {
                    time_since_ball_seen = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
                                                              NUClear::clock::now() - ball->time_of_measurement)
                                                              .count()
                                                          / 1000.0);
                    ball_is_visible      = "is currently visible";
                    Eigen::Vector3d rBRr = sensors.Hrw * ball->rBWw;
                    distance_to_ball     = std::to_string(rBRr.norm());
                }


                // Build prompt for LLM
                std::string prompt = "Given desired user request: " + cfg.user_request + " and current information of the world: " + "Ball " + ball_is_visible + " , last seen " + time_since_ball_seen + " seconds ago" + distance_to_ball + " m away from you. You have the ability to WalkToBall, Kick, LookAround, LookAtBall, StandStill, TurnOnSpot. What tasks should you currently do to achieve the request? \n Provide your response as a list of with format: Task: <task> Priority: <priority> \n where <task> is one of the aforementioned tasks, and <priority> is an integer above 0, a higher number means a higher priority and will be take control over a lower priority tasks servos.Only use the tasks required to achieve the request. Use the skills to navigate your environment to achieve the task. Only once you have achieved the task: + " + cfg.user_request + " , respond with StandStill with the highest priority. \n";
                log<NUClear::INFO>("Prompt is:\n", prompt);
                // Send request to OpenAI API
                nlohmann::json request = {
                    {"model", "gpt-3.5-turbo"},
                    {"messages", nlohmann::json::array({{{"role", "user"}, {"content", prompt}}})},
                    {"max_tokens", 100},
                    {"temperature", 0}};
                //   {"prompt", prompt},
                auto chat = utility::openai::chat().create(request);

                // auto response = completion["choices"][0]["text"].get<std::string>();
                auto response = chat["choices"][0]["message"]["content"].get<std::string>();
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
                        emit<Task>(std::make_unique<WalkToFieldPosition>(
                                       Eigen::Vector3f(cfg.walk_to_field_position_position.x(),
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
                    else if (task == "TurnOnSpot") {
                        emit<Task>(std::make_unique<TurnOnSpot>(), priority);
                    }
                    else {
                        log<NUClear::ERROR>("Unexpected task from LLM response: ", task);
                    }
                }


                // Profit.
            });
    }

}  // namespace module::purpose
