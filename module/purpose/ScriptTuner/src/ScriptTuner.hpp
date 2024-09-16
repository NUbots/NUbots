/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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

#ifndef MODULES_BEHAVIOUR_TOOLS_SCRIPTTUNER_HPP
#define MODULES_BEHAVIOUR_TOOLS_SCRIPTTUNER_HPP

#include <filesystem>
#include <nuclear>

#include "message/actuation/Limbs.hpp"

#include "utility/skill/Script.hpp"

namespace module::purpose {

    using message::actuation::BodySequence;
    using utility::skill::Script;

    /**
     * Provides a Curses interface to let the user customize scripts
     *
     * @author Trent Houliston
     */
    class ScriptTuner : public NUClear::Reactor {
    private:
        /// @brief The autosave interval in seconds
        /// @note Set to 0 to disable autosave
        static constexpr size_t AUTOSAVE_INTERVAL = 30;

        /// @brief Is autosaving enabled?
        bool autosave_enabled = (AUTOSAVE_INTERVAL != 0);

        /// @brief The autosave directory, imported from the config file
        std::filesystem::path autosave_dir = "";

        /// @brief The path to the script we are editing
        std::filesystem::path script_path;

        /// @brief The script object we are editing
        Script<BodySequence> script;

        /// @brief The index of the frame we are currently editing
        size_t frame;

        /// @brief The index of the item we are selecting
        size_t selection;

        /// @brief If we are selecting the angle or gain for this item
        /// @note angle = true, gain = false
        bool angle_or_gain = true;

        /// @brief If we are displaying angles in degrees or radians
        /// @note degrees = true, radians = false
        bool deg_or_rad = false;

        /// @brief Whether changes have been made to the script since last save
        bool unsaved_changes = false;

        /// @brief The last autosave location, so we can notify the user
        std::filesystem::path autosave_path = "";

        /// @brief The time that ScriptTuner was started
        /// @note Used to determine which autosaves are safe to delete
        std::chrono::system_clock::time_point start_time;

        /// @brief Default gain for new frames
        const double default_gain = 6.64;

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

        /// @brief Setup up colour pairs for ncurses
        void setup_colour_pairs();

        /// @brief Print an ascii image of the NUgus with labelled motors
        /// @param rows Starting position of the image (top left)
        /// @param cols Starting position of the image (top left)
        void print_nugus(const size_t rows, const size_t cols);

        /// @brief Change selection up, down, left, right from nugus' body
        /// @param direction The direction to move the selection (up, left, down, right)=(0, 1, 2, 3)
        void move_nugus_selection(const uint8_t direction);

    public:
        explicit ScriptTuner(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::purpose

#endif  // MODULE_PURPOSE_SCRIPTTUNER_HPP
