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
    using message::onboarding::Ping;
    using message::onboarding::Pong;
    using message::onboarding::Task;

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
        if(pong_msg.judgeMe == 1)
        {
            // Read final answer
            float ans = pong_msg.finalAnswer;
            // Check if soln is correct
            if(ans == 55)
            {
                // Set isCorrect flag to true
                task_msg->isCorrect = 1;
                // log final answer as str
                log<NUClear::INFO>("Final answer:", ans);
            }
            else
            {
                // Log that provided soln was incorrect
                log<NUClear::INFO>("Incorrect solution");
            }
        }
        else
        {
            // Do nothing
        }

    });
}

}  // namespace module::onboarding
