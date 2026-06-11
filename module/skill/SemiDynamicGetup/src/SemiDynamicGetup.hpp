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
            // --- Script selection ---
            std::vector<std::string> getup_back{};
            std::vector<std::string> getup_stand{};

            // --- Rise phase ---
            int rise_frame_count   = 3;
            double pitch_reference = 0.7;
            double Kp              = 0.3;
            double max_correction  = 0.3;

            // --- Fall detection ---
            double fall_recovery_tilt  = 1.0;  // rad
            double fall_gyro_threshold = 2.0;  // rad/s
            int fall_debounce_ticks    = 3;

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

        std::vector<utility::skill::Frame> early_frames{};
        std::vector<utility::skill::Frame> rise_frames{};

        NUClear::clock::time_point stabilisation_start{};

        /// Consecutive sensor updates the mid-script fall condition has held for
        int fall_tick_count = 0;
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
