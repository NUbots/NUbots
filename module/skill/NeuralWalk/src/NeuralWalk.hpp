/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 */
#ifndef MODULE_SKILL_NEURALWALK_HPP
#define MODULE_SKILL_NEURALWALK_HPP

#include <nuclear>
#include <openvino/openvino.hpp>
#include <vector>

#include "extension/Behaviour.hpp"

#include "message/actuation/ServoCommand.hpp"
#include "message/behaviour/state/WalkState.hpp"
#include "message/skill/Walk.hpp"

#include "utility/input/ServoID.hpp"

namespace module::skill {

    class NeuralWalk : public ::extension::behaviour::BehaviourReactor {
    private:
        // OpenVINO inference objects
        ov::CompiledModel compiled_model;
        ov::InferRequest infer_request;

        // Configuration
        struct Config {
            std::string model_path;
            std::string device;
            double step_period;
            Eigen::Vector3d acceleration = Eigen::Vector3d(0.5, 0.5, 0.5); // Default acceleration
            std::map<utility::input::ServoID, message::actuation::ServoState> servo_states;
            std::vector<std::pair<utility::input::ServoID, double>> arm_positions;
        } cfg;

        // Current smoothed velocity
        Eigen::Vector3d current_velocity = Eigen::Vector3d::Zero();

        NUClear::clock::time_point last_update_time;

        // Autoregressive action history (3 frames * 12 leg joints)
        std::vector<float> action_history;
        
        // Internal clock for generating phase indicators
        double phase_time = 0.0;
        double phase_indicator = 1.0;

    public:
        /// @brief Called by the powerplant to build and setup the NeuralWalk reactor.
        explicit NeuralWalk(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::skill

#endif  // MODULE_SKILL_NEURALWALK_HPP
