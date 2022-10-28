#ifndef MODULE_MOTION_SERVOS_HPP
#define MODULE_MOTION_SERVOS_HPP

#include <nuclear>

#include "extension/Behaviour.hpp"

#include "message/input/Sensors.hpp"
#include "message/motion/ServoTarget.hpp"

#include "utility/input/ServoID.hpp"
#include "utility/motion/ServoLimbMap.hpp"

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

        /// @brief Creates a reaction that takes a servo wrapper task (eg LeftLeg) and emits a task for each servo.
        /// Emits Done when all servo tasks are Done.
        /// @tparam Group is a servo wrapper task (eg LeftLeg)
        /// @tparam Elements is a template pack of Servos that Group uses
        template <typename Group, typename... Elements>
        void add_group_provider() {
            on<Provide<Group>, Needs<Elements>...>().then(
                [this](const Group& group, const RunInfo& info, const Uses<Elements>... elements) {
                    // Check if any subtask is Done
                    if (info.run_reason == RunInfo::RunReason::SUBTASK_DONE) {
                        // If every servo task is done then emit Done
                        if ((elements.done && ...)) {
                            emit<Task>(std::make_unique<Done>());
                            return;
                        }
                        // Emit Idle if all the servos are not Done yet
                        emit<Task>(std::make_unique<Idle>());
                        return;
                    }

                    // Runs an emit for each servo
                    NUClear::util::unpack((emit<Task>(std::make_unique<Elements>(
                                               group.servos[utility::motion::ServoMap<Elements>::value])),
                                           0)...);
                });
        }

    public:
        /// @brief Called by the powerplant to build and setup the Servos reactor.
        explicit Servos(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::motion

#endif  // MODULE_MOTION_SERVOS_HPP
