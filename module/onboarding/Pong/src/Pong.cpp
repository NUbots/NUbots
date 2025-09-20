#include "Pong.hpp"

#include "extension/Configuration.hpp"

#include "message/onboarding/Ping.hpp"
#include "message/onboarding/Pong.hpp"

namespace module::onboarding {

    using extension::Configuration;

    int TARGET = 50;

    bool pre_check(int lower_bound, int upper_bound) {
        // ensure valid range
        if (lower_bound >= upper_bound) {
            return false;
        }
        return true;
    }

    std::string fizz_buzz(int number) {
        // fizz, buzz, fizzbuzz?
        if (number % 3 == 0 && number % 5 == 0) {
            return "FizzBuzz";
        }
        else if (number % 3 == 0) {
            return "Fizz";
        }
        else if (number % 5 == 0) {
            return "Buzz";
        }
    }

    void recursive_iterate(int current, int target) {
        // tail led direct recursion
        if (current > target) {
            // robot nod here
            return;
        }
        if (current % 3 == 0 || current % 5 == 0) {
            std::cout << fizz_buzz(current) << std::endl;
        }
        else {
            std::cout << current << std::endl;
        }
        recursive_iterate(current + 1, target);
    }


    Pong::Pong(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        using message::onboarding::Ping;
        using message::onboarding::Pong;

        on<Configuration>("Pong.yaml").then([this](const Configuration& config) {
            // Use configuration here from file Pong.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then([this] {
            // Start the ping pong chain
            auto pong_msg = std::make_unique<Pong>();
            // std::cout << "[DEBUG] Pre-check result: " << pre_check(1, TARGET) << std::endl;
            if (pre_check(1, TARGET)) {
                recursive_iterate(1, TARGET);
            }

            //  emit(pong_msg);
        });

        on<Trigger<Ping>>().then([this](const Ping& ping_msg) {
            // Log a INFO level message with the text "Pong"
            std::string pong_text = "Pong";
            log<INFO>(pong_text);

            // Emit a Pong message
            auto pong_msg = std::make_unique<Pong>();
            emit(pong_msg);
        });
    }

}  // namespace module::onboarding
