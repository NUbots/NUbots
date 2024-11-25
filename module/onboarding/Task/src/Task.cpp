#include "Task.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/Ping.hpp"
#include "message/onboarding/Pong.hpp"
#include "message/onboarding/Task.hpp"

namespace module::onboarding
{

using extension::Configuration;

Task::Task(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment))
{

    on<Configuration>("Task.yaml").then([this](const Configuration& config)
    {
        // Use configuration here from file Task.yaml
        this->log_level = config["log_level"].as<NUClear::LogLevel>();
    });

    on<Startup>().then([this]
    {
        // Vibe
    });

    // Check if final sum is correct
    on<Trigger<Pong>>().then([this](const Pong& pong_msg)
    {
        auto task_msg = std::make_unique<Task>();
        // Check judgeMe flag from pong_msg
        if(pong_msg->judgeMe == 1)
        {
            // Read final answer
            // str ans = pong_msg->finalAnswer;
            // log final answer as str
            log<NUClear::INFO>("%f", pong_msg->finalAnswer);
        }


    });
}

}  // namespace module::onboarding
