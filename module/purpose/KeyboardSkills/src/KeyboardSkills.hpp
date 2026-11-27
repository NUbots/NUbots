/*
 * MIT License
 *
 * Copyright (c) 2009 NUbots
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
#ifndef MODULE_PURPOSE_KEYBOARDSKILLS_HPP
#define MODULE_PURPOSE_KEYBOARDSKILLS_HPP

#include <Eigen/Core>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// clang-format off
// This include needs to come immediately before the OK undef
#include <ncurses.h>
// because ncurses defines OK. We don't need (or want) it.
#undef OK
// clang-format on

#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    /// Plays one K1 from the keyboard with the mjlab policies: the walk and the six ball skills that
    /// skill::Walk provides. The NUbots counterpart of NUSim's `./b skills`.
    class KeyboardSkills : public ::extension::behaviour::BehaviourReactor {
    public:
        explicit KeyboardSkills(std::unique_ptr<NUClear::Environment> environment);

        /// What the keys drive: the walk or one of the ball skills, in BallSkill::Type order after WALK
        enum class Mode { WALK, RECEIVE, DRIBBLE, KICK, KICK_AIR, FIRST_TOUCH, FIRST_PASS };

    private:
        struct Config {
            /// Walk command step per key press: (vx, vy) m/s and wz rad/s
            double walk_step = 0.1;
            double turn_step = 0.2;
            /// Ball velocity step per key press (m/s), and the command each ball skill starts with
            double ball_step              = 0.2;
            Eigen::Vector2d dribble_start = Eigen::Vector2d(0.0, 0.0);
            Eigen::Vector2d kick_start    = Eigen::Vector2d(4.0, 0.0);
            Eigen::Vector2d touch_start   = Eigen::Vector2d(0.0, 0.9);
            Eigen::Vector2d pass_start    = Eigen::Vector2d(-3.0, 0.0);
            /// Head step per key press (rad), for the look in walk mode
            double head_step = 2.0 * M_PI / 180.0;
            /// The pass `p` rolls at the robot: from this far ahead, arriving at this speed, with this rolling
            /// deceleration (m/s^2) to work out the launch speed
            double pass_distance = 3.0;
            double pass_arrival  = 2.0;
            double pass_decel    = 0.9;
            /// Where `b` puts the ball at rest, m ahead
            double drop_distance = 0.5;
            /// Read keys from the terminal. Otherwise only the script plays.
            bool interactive = true;
            /// Hold ball commands fixed in the world while the robot turns, rather than turning with it
            bool hold_in_world = true;
            /// Key presses at given times after startup (s), for running without a terminal
            std::vector<std::pair<double, int>> script{};
        } cfg;

        Mode mode = Mode::WALK;
        /// Walk twist (vx, vy, wz)
        Eigen::Vector3d walk_command = Eigen::Vector3d::Zero();
        /// Ball velocity command, in the odometry world {w} when held in the world, else the robot yaw frame
        Eigen::Vector2d ball_command = Eigen::Vector2d::Zero();
        /// The robot's heading in the odometry world {w}
        double yaw = 0.0;

        /// The ball command in the robot yaw frame, as the skill takes it
        [[nodiscard]] Eigen::Vector2d ball_command_robot() const;
        /// Sets the ball command from one in the robot yaw frame
        void set_ball_command_robot(const Eigen::Vector2d& command);
        double head_yaw   = 0.0;
        double head_pitch = 0.0;

        /// Next script entry to play, and when the script started
        std::size_t script_next = 0;
        NUClear::clock::time_point started{};

        /// Reads keys from the terminal, enabled only when interactive
        ReactionHandle terminal_reader{};
        bool curses          = false;
        bool colours_enabled = false;
        std::shared_ptr<WINDOW> command_window{};
        std::shared_ptr<WINDOW> log_window{};
        std::mutex mutex;

        /// Acts on one key press
        void key(int k);
        /// Switches what the keys drive, setting the command the new skill starts with
        void set_mode(Mode m);
        /// Emits the task for the current mode and command, and removes the other
        void update_command();
        /// Rolls a pass at the robot in NUSim
        void roll_pass();
        /// Puts the ball at rest in front of the robot in NUSim
        void drop_ball();

        void start_curses();
        void print_status();
        void write(const std::shared_ptr<WINDOW>& window, short colour, const std::string& text);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_KEYBOARDSKILLS_HPP
