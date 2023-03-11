#ifndef MODULE_PURPOSE_ROBOCUP_HPP
#define MODULE_PURPOSE_ROBOCUP_HPP

#include <nuclear>
#include <string>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    class RoboCup : public ::extension::behaviour::BehaviourReactor {
    private:
        // Smart enum for the robot's position
        struct Position {
            enum Value {
                STRIKER,
                GOALIE,
                DEFENDER,
            };
            Value value = Value::STRIKER;

            Position() = default;
            Position(std::string const& str) {
                // clang-format off
                if (str == "STRIKER") { value = Value::STRIKER; }
                else if (str == "GOALIE") { value = Value::GOALIE; }
                else if (str == "DEFENDER") { value = Value::DEFENDER; }
                else { throw std::runtime_error("Invalid robot position"); }
                // clang-format on
            }

            operator int() const {
                return value;
            }
        };

        /// @brief Stores configuration values
        struct Config {
            bool force_playing          = false;
            bool force_penalty_shootout = false;
            Position position{};
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the RoboCup reactor.
        explicit RoboCup(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_ROBOCUP_HPP
