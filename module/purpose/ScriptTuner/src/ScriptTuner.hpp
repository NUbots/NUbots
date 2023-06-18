/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2023 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_TOOLS_SCRIPTTUNER_HPP
#define MODULES_BEHAVIOUR_TOOLS_SCRIPTTUNER_HPP

#include <nuclear>

#include "message/actuation/Limbs.hpp"

#include "utility/skill/Script.hpp"

namespace module::purpose {

    using message::actuation::LimbsSequence;
    using utility::skill::Script;

    /**
     * Provides a Curses interface to let the user customize scripts
     *
     * @author Trent Houliston
     */
    class ScriptTuner : public NUClear::Reactor {
    private:
        /// @brief The path to the script we are editing
        std::string script_path;

        /// @brief The script object we are editing
        Script<LimbsSequence> script;

        /// @brief The index of the frame we are currently editing
        size_t frame;

        /// @brief The index of the item we are selecting
        size_t selection;

        /// @brief If we are selecting the angle or gain for this item
        bool angle_or_gain;

        /// @brief Default gain for new frames
        const size_t default_gain = 10;

        /// @brief Default duration for new frames
        const size_t default_duration = 1000;

        /// @brief Listens for user input
        static std::string user_input();

        /// @brief Refreshes the ncurses view
        void refresh_view();

        /// @brief Loads in the script from the specified path
        void load_script(const std::string& path);

        /// @brief Saves the current script
        void save_script();

        /// @brief Edits the duration of the current frame
        void edit_duration();

        /// @brief Edits the angle of the current selection
        void edit_selection();

        /// @brief Emits targets of the current frame
        void activate_frame(int frame);

        /// @brief
        void toggle_lock_motor();

        /// @brief Adds a new frame to the script
        void new_frame();

        /// @brief Deletes the current frame from the script
        void delete_frame();

        /// @brief Plays the script
        void play_script();

        /// @brief Jumps to the specified frame without moving the robot
        void jump_to_frame();

        /// @brief Prints a list of commands to the screen
        void help();

        /// @brief
        void edit_gain_input();

        /// @brief Switches angle and gains between corresponding left and right motors, flips script around z axis
        void mirror_script();

        /// @brief Change script_path and then call save_script to Save As
        void save_script_as();

        /// @brief Allows user to edit the gain for the entire script or specified frame
        void edit_gain();

        /// @brief Checks user input is a number and converts it a number that becomes the new frame number
        void user_input_to_frame();

        /// @brief Converts valid user input to gain
        static float user_input_to_gain();

    public:
        explicit ScriptTuner(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::purpose

#endif  // MODULE_PURPOSE_SCRIPTTUNER_HPP
