#include "Ping.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/Ping.hpp"
#include "message/onboarding/Pong.hpp"
#include "message/onboarding/FoundAnswer.hpp"

namespace module::onboarding {

    using extension::Configuration;

    Ping::Ping(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        using message::onboarding::Ping;
        using message::onboarding::Pong;
        using message::onboarding::FoundAnswer;

        on<Configuration>("Ping.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Ping.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then([this] {
            // Vibe
        });

        on<Trigger<Pong>>().then([this](const Pong& pong_msg) {
            auto ping_msg = std::make_unique<Ping>();

            // Iterate the counters and track it
            ping_msg->count = pong_msg.count + 1;
            ping_msg->iter_count = ping_msg->count + pong_msg.iter_count;

            if(ping_msg->iter_count == 55){
                log<INFO>("Ping count: ", ping_msg->count, " Found Answer! ", ping_msg->iter_count);

                auto found_answer_msg = std::make_unique<FoundAnswer>();
                emit(found_answer_msg);
            }
            else{
                log<INFO>("Ping count: ", ping_msg->count, " Ping iter_count: ", ping_msg->iter_count);
                emit(ping_msg);
            }

        });



        /*
        on<Trigger<Judge>>().then([this](const Pong& pong_msg) {
            auto ping_msg = std::make_unique<Ping>();
            pingCounter += pong_msg.count;
            ping->count = pingCounter;
            log<INFO>("Ping");
            emit(ping_msg);
        });
        */
    }

}  // namespace module::onboarding
