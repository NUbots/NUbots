#ifndef MODULE_TOOLS_ROBOCUPCONFIGURATION_HPP
#define MODULE_TOOLS_ROBOCUPCONFIGURATION_HPP

#include <nuclear>
#include <string>

namespace module::tools {

    class RoboCupConfiguration : public NUClear::Reactor {
    private:
        std::string hostname   = "";
        std::string ip_address = "";
        int team_id            = 0;
        int player_id          = 0;

        /// @brief Smart enum for the robot's position
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

            Position::operator std::string() const {
                switch (value) {
                    case Value::STRIKER: return "Striker";
                    case Value::DEFENDER: return "Defender";
                    case Value::GOALIE: return "Goalie";
                    default: throw std::runtime_error("enum Position's value is corrupt, unknown value stored");
                }

                operator int() const {
                    return value;
                }
            }
            robot_position;


            void refresh_view();

        public:
            /// @brief Called by the powerplant to build and setup the RoboCupConfiguration reactor.
            explicit RoboCupConfiguration(std::unique_ptr<NUClear::Environment> environment);
        };

    }  // namespace module::tools

#endif  // MODULE_TOOLS_ROBOCUPCONFIGURATION_HPP
