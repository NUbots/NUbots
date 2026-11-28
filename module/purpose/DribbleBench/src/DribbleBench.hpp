/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 */
#ifndef MODULE_PURPOSE_DRIBBLEBENCH_HPP
#define MODULE_PURPOSE_DRIBBLEBENCH_HPP

#include <Eigen/Geometry>
#include <map>
#include <nuclear>
#include <random>
#include <string>
#include <vector>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    /// Runs mjlab's dribble benchmarks (straight-line speed, figure of eight, walk -> dribble) on one robot in
    /// NUSim, through skill::Walk's ball skills, and prints the results. NUSim only: it steers and scores with
    /// the simulator's ground truth.
    class DribbleBench : public ::extension::behaviour::BehaviourReactor {
    public:
        explicit DribbleBench(std::unique_ptr<NUClear::Environment> environment);

    private:
        enum class Kind { STRAIGHT, FIGURE8, WALK_TO_DRIBBLE };
        enum class Phase { RECOVER, FACE, SETTLE, PLACE, HOLD, APPROACH, RUN, DONE };

        struct Trial {
            Kind kind;
            double value;  // speed (straight) or radius (figure of eight)
        };

        struct Config {
            std::vector<double> speeds{};
            int straight_repeats = 2;
            std::vector<double> radii{};
            int figure8_repeats           = 2;
            double figure8_circle_timeout = 40.0;
            int walk_repeats              = 4;
            unsigned seed                 = 1;
        } cfg;

        std::vector<Trial> trials{};
        std::size_t trial = 0;
        Phase phase       = Phase::RECOVER;
        double phase_t    = 0.0;  // s in the current phase
        std::mt19937 rng{1};

        // Ground truth, simulator world {s}
        Eigen::Vector2d robot{};
        double robot_z   = 0.5;
        double robot_yaw = 0.0;
        Eigen::Vector2d ball{};
        bool have_truth = false;
        bool fallen     = false;
        double upright_t = 0.0;

        // The trial in progress
        Eigen::Vector2d direction{};  // world command direction
        Eigen::Vector2d origin{};     // figure of eight: the ball start
        double frame_yaw  = 0.0;      // figure of eight: the robot's heading at the start
        bool lost         = false;
        bool fell         = false;
        std::string why{};
        double along_start = 0.0;
        double switch_at   = 0.6;
        double walk_t      = 0.0;
        // Figure of eight
        int side         = 0;
        double swept     = 0.0;
        double prev_angle = 0.0;
        bool fresh_circle = true;
        double circle_t   = 0.0;
        double err_max    = 0.0;
        int circles_done  = 0;

        // Results: name -> list of rows
        std::map<std::string, std::vector<std::vector<double>>> results{};

        void tick();
        void start_trial();
        void end_trial();
        void set_phase(Phase p);
        void walk(double vx, double wz);
        void dribble(const Eigen::Vector2d& command_world);
        void place_ball(double distance, double bearing);
        void summary();
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_DRIBBLEBENCH_HPP
