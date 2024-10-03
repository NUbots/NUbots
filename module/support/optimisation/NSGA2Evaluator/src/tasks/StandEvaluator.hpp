/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef MODULE_SUPPORT_OPTIMISATION_STANDEVALUATOR_HPP
#define MODULE_SUPPORT_OPTIMISATION_STANDEVALUATOR_HPP

#include <Eigen/Core>
#include <nuclear>
#include <vector>

#include "EvaluatorTask.hpp"

#include "message/input/Sensors.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

namespace module::support::optimisation {
    using message::input::Sensors;
    using message::platform::RawSensors;
    using message::support::optimisation::NSGA2EvaluationRequest;
    using message::support::optimisation::NSGA2FitnessScores;

    class StandEvaluator : public EvaluatorTask {
    public:
        // Implementing the EvaluatorTask interface
        void process_optimisation_robot_position(const OptimisationRobotPosition& position);
        void set_up_trial(const NSGA2EvaluationRequest& request);
        void reset_trial();
        void evaluating_state(NSGA2Evaluator* evaluator);
        std::unique_ptr<NSGA2FitnessScores> calculate_fitness_scores(bool early_termination,
                                                                     int generation,
                                                                     int individual);

        // Task-specific functions
        std::vector<double> calculate_scores(double trial_duration);
        std::vector<double> calculate_constraints(bool early_termination);
        bool has_fallen(const Sensors& sensors);
        void update_max_field_plane_sway(const Sensors& sensors);

    private:
        struct Config {
            /// @brief Threshold angle for fallover detection, between torso z axis and world z axis
            float fallen_angle = 0.0f;
        } cfg;

        /// @brief Robot state for this evaluation, used during fitness and constraint calculation
        Eigen::Vector3d robot_position = Eigen::Vector3d::Zero();
        double max_field_plane_sway    = 0.0;
        RawSensors current_sensors;

        /// @brief The amount of time to run a single trial, in seconds.
        std::chrono::seconds trial_duration_limit = std::chrono::seconds(0);

        /// @brief Keep track of when the trial started
        NUClear::clock::time_point trial_start_time{NUClear::clock::now()};

        void load_script(std::string script_path);
        void save_script(std::string script_path);

        std::string script;
        void run_script(NSGA2Evaluator* evaluator);

        /// @brief Indicates that a fall has occured.
        bool fallen = false;
    };

}  // namespace module::support::optimisation

#endif  // MODULE_SUPPORT_OPTIMISATION_STANDEVALUATOR_HPP
