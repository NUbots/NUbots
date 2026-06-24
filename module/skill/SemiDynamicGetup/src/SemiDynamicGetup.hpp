#ifndef MODULE_SKILL_SEMIDYNAMICGETUP_HPP
#define MODULE_SKILL_SEMIDYNAMICGETUP_HPP

#include <nuclear>
#include <string>
#include <vector>

#include "extension/Behaviour.hpp"

#include "message/actuation/Limbs.hpp"

#include "utility/skill/Script.hpp"

namespace module::skill {

    class SemiDynamicGetup : public ::extension::behaviour::BehaviourReactor {
    private:
        struct Config {
            // --- Per-direction getup (front / back) ---
            // Everything that differs between rising from the front and the back:
            // the script(s) to play, how many trailing frames form the balance-
            // corrected rise phase, and the rise-phase tuning.
            struct DirectionConfig {
                std::vector<std::string> scripts{};  // getup script(s) for this direction
                int rise_frame_count   = 3;
                double pitch_reference = 0.7;
                double Kp              = 0.3;
                double max_correction  = 1.0;
            };
            DirectionConfig back{};
            DirectionConfig front{};

            // Stand script, shared by both directions (run after the rise phase).
            std::vector<std::string> getup_stand{};

            // --- Fall detection (ported from FallingRelaxPlanner) ---
            // Settled-state tilt check, used at the end of RISE/STAND to decide
            // whether the getup ended with the robot upright (high tilt, low motion).
            double fall_recovery_tilt = 1.0;  // rad

            // Per-signal thresholds for the dynamic falling detector. Each signal is
            // exponentially smoothed, then classified STABLE / UNSTABLE / FALLING.
            struct Levels {
                double mean      = 0.0;  // value subtracted from the raw reading
                double unstable  = 0.0;  // threshold to be considered unstable
                double falling   = 0.0;  // threshold to be considered falling
                double smoothing = 0.0;  // exponential filter factor (weight on the previous value)
            };
            Levels gyro_mag{};   // sum of |rotation rate| across the three axes
            Levels acc_mag{};    // accelerometer magnitude
            Levels acc_angle{};  // accelerometer angle from upright

            // --- Foot polygon geometry (should match KinematicsConfiguration.yaml) ---
            // Origin assumed at ankle joint centre in the foot frame (x-forward, y-left).
            double foot_toe_length  = 0.130;  // m, ankle to toe tip
            double foot_heel_length = 0.085;  // m, ankle to heel
            double foot_width       = 0.130;  // m, total foot width

            // --- Capture-point stabilisation ---
            double max_stabilisation_time = 5.0;   // s
            double cp_margin              = 0.02;  // m, CP must be this far inside polygon to be stable
            double cp_velocity_gain       = 1.5;   // (m/s) / m of CP offset
            double max_step_velocity      = 0.2;   // m/s, clamp on walk command magnitude
            double walk_command_rate      = 10.0;  // Hz, max rate of corrective Walk task emission
        } cfg;

        // EARLY:     run the first (n - rise_frame_count) frames unmodified
        // RISE:      run the last rise_frame_count frames with a one-shot pitch correction on hip pitch
        // STAND:     run Stand.yaml
        // STABILISE: walk to bring the Capture Point inside the support polygon
        enum class InternalPhase { EARLY, RISE, STAND, STABILISE };
        InternalPhase internal_phase = InternalPhase::EARLY;

        /// Which orientation this getup started from, set from the task message at NEW_TASK.
        /// Selects the script frames, rise tuning and the "still in start orientation" test.
        enum class StartSide { BACK, FRONT };
        StartSide start_side = StartSide::BACK;

        /// Per-signal stability categorisation for the dynamic falling detector.
        enum class State { STABLE, UNSTABLE, FALLING };

        /// Current exponentially smoothed value of each falling-detection signal.
        double gyro_mag_value  = 0.0;
        double acc_mag_value   = 0.0;
        double acc_angle_value = 0.0;

        /// Exponential filter: weight `alpha` on the previous value, `1 - alpha` on the new value.
        template <typename Scalar>
        Scalar smooth(Scalar value, Scalar new_value, Scalar alpha) {
            return alpha * value + (1.0 - alpha) * new_value;
        }

        /// EARLY + RISE frame split for one getup direction, precomputed at config load.
        struct GetupFrames {
            std::vector<utility::skill::Frame> early{};
            std::vector<utility::skill::Frame> rise{};
        };
        GetupFrames back_frames{};
        GetupFrames front_frames{};

        /// The frames / tuning for the currently active getup direction.
        const GetupFrames& active_frames() const {
            return start_side == StartSide::FRONT ? front_frames : back_frames;
        }
        const Config::DirectionConfig& active_cfg() const {
            return start_side == StartSide::FRONT ? cfg.front : cfg.back;
        }

        NUClear::clock::time_point stabilisation_start{};

        /// When the last corrective Walk task was emitted (for rate limiting)
        NUClear::clock::time_point last_walk_command{};

        /// Build a BodySequence from a frame list, optionally adding hip_correction (rad) to both hip pitch joints.
        std::unique_ptr<message::actuation::BodySequence> make_sequence(
            const std::vector<utility::skill::Frame>& frames,
            double hip_correction = 0.0) const;

    public:
        explicit SemiDynamicGetup(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_SEMIDYNAMICGETUP_HPP
