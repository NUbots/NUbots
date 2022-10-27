#ifndef MODULE_MOTION_SERVOS_HPP
#define MODULE_MOTION_SERVOS_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

#include "message/input/Sensors.hpp"
#include "message/motion/ServoTarget.hpp"

#include "utility/input/ServoID.hpp"

namespace module::motion {
    using message::input::Sensors;
    using message::motion::ServoTarget;
    using utility::input::ServoID;

    class Servos : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Creates a reaction that sends a given servo command as a servo target command for the platform module
        /// to use
        // TODO(ysims): add capability to be Done when the servo reaches the target position
        template <typename Servo, ServoID::Value ID>
        void add_servo_provider() {
            on<Provide<Servo>, Trigger<Sensors>>().then([this](const Servo& servo) {
                // If the time to reach the position is over, then stop requesting the position
                if (NUClear::clock::now() >= servo.command.time) {
                    emit<Task>(std::make_unique<Done>());
                    return;
                }
                emit(std::make_unique<ServoTarget>(servo.command.time,
                                                   ID,
                                                   servo.command.position,
                                                   servo.command.state.gain,
                                                   servo.command.state.torque));
            });
        }

    public:
        /// @brief Called by the powerplant to build and setup the Servos reactor.
        explicit Servos(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::motion

#endif  // MODULE_MOTION_SERVOS_HPP
