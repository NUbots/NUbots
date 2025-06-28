#ifndef MODULE_SKILL_SEMIDYNAMICGETUP_HPP
#define MODULE_SKILL_SEMIDYNAMICGETUP_HPP

#include <nuclear>

namespace module::skill {

class SemiDynamicGetup : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {

    } cfg;

    enum class Phase {
        LAYING,
        ARM_PUSH,
        TORSO_SWING,
        KNEE_TUCK,
        RISE_TO_CROUCH,
        STAND_UP
    };

    Phase current_phase = Phase::LAYING;

    float last_pitch = 0.0f;
    float pitch_velocity = 0.0f;

    // Time tracking
    NUClear::clock::time_point phase_start;

    // PID gains
    float Kp = 5.0f;    // TODO
    float Kd = 1.0f;    // TODO

    void update_pitch(float new_pitch);
    void transition_to_phase(Phase new_phase);

public:
    /// @brief Called by the powerplant to build and setup the SemiDynamicGetup reactor.
    explicit SemiDynamicGetup(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::skill

#endif  // MODULE_SKILL_SEMIDYNAMICGETUP_HPP
