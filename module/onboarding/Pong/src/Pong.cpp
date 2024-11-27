#include "Pong.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/Ping.hpp"
#include "message/onboarding/Pong.hpp"
#include "message/onboarding/Task.hpp"

namespace module::onboarding
{

    using extension::Configuration;

    Pong::Pong(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment))
    {

        using message::onboarding::Ping;
        using message::onboarding::Pong;
        using message::onboarding::Task;

        on<Configuration>("Pong.yaml").then([this](const Configuration& config)
        {
            // Use configuration here from file Pong.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.nFinal      = config["answer"].as<int>();
        });

        on<Startup>().then([this]
        {
            // Vibe (The ping module will start the reaction)

        });

        on<Trigger<Ping>>().then([this](const Ping& ping_msg)
        {
            // Log a INFO level message with the text "Pong"
            auto pong_msg = std::make_unique<Pong>();
            log<NUClear::INFO>("Pong");

            // Read k value from ping & check if k = n+1
            if(ping_msg.k == cfg.nFinal + 1)
            {
                // Stop computing sum
                pong_msg->judgeMe = 1;

                // Define final answer
                pong_msg->finalAnswer = ping_msg.tempSum;
            }
            else
            {
                // Return the temporary sum recieved from ping
                pong_msg->tempSum = ping_msg.tempSum;
                pong_msg->k = ping_msg.k;
            };

            // Read (and set) tempSum from pin
            // pong_msg.tempSum = ping_msg.tempSum;

            // Emit a Pong message
            emit(pong_msg);
        });
    }

}  // namespace module::onboarding
