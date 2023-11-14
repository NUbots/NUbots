#include "Pong.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/Judge.hpp"
#include "message/onboarding/Ping.hpp"
#include "message/onboarding/Pong.hpp"


namespace module::onboarding {

    using extension::Configuration;


    Pong::Pong(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        using message::onboarding::Judge;
        using message::onboarding::Ping;
        using message::onboarding::Pong;

        on<Configuration>("Pong.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Pong.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.max_iter    = config["max_iter"].as<int>();
        });

        on<Trigger<Ping>>().then([this](const Ping& ping_msg) {
            auto pong_msg = std::make_unique<Pong>();
            pong_msg->val = ping_msg.val + ping_msg.iter_count + 1;

            int curr_iter        = ping_msg.iter_count + 1;
            pong_msg->iter_count = curr_iter;

            log<NUClear::INFO>(pong_msg->val);
            log<NUClear::INFO>(pong_msg->iter_count);

            if (curr_iter >= cfg.max_iter) {
                auto judge_msg = std::make_unique<Judge>();
                judge_msg->val = pong_msg->val;
                emit(judge_msg);
            }
            else {
                emit(pong_msg);
            }
        });
    }

}  // namespace module::onboarding
