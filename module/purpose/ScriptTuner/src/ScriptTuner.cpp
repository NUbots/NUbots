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

#include "ScriptTuner.hpp"

extern "C" {
#include <ncurses.h>
#undef OK
}

#include <cstdio>
#include <fcntl.h>
#include <sstream>
#include <termios.h>

#include "extension/Behaviour.hpp"
#include "extension/Configuration.hpp"

#include "message/actuation/ServoTarget.hpp"
#include "message/behaviour/state/Stability.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/strategy/FallRecovery.hpp"
#include "message/strategy/StandStill.hpp"

#include "utility/file/fileutil.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/angle.hpp"
#include "utility/platform/RawSensors.hpp"

namespace module::purpose {

    using NUClear::message::CommandLineArguments;

    using message::actuation::BodySequence;
    using message::actuation::ServoTarget;
    using message::actuation::ServoTargets;
    using message::behaviour::state::Stability;
    using message::platform::RawSensors;
    using message::strategy::FallRecovery;
    using message::strategy::StandStill;

    using extension::Configuration;
    using extension::behaviour::Task;

    using utility::skill::Frame;
    using utility::skill::Script;
    using LimbID  = utility::input::LimbID;
    using ServoID = utility::input::ServoID;

    struct LockServo {};

    ScriptTuner::ScriptTuner(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), script_path("Initializing..."), frame(0), selection(0), angle_or_gain(true) {

        // Add a blank frame to start with
        script.frames.emplace_back();
        script.frames.back().duration = std::chrono::milliseconds(default_duration);

        on<Configuration>("ScriptTuner.yaml").then([this](const Configuration& config) {
            // Use configuration here from file KeyboardWalk.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Startup>().then([this] {
            // Start curses mode
            initscr();
            // Capture our characters immediately (but pass through signals)
            cbreak();
            // Capture arrows and function keys
            keypad(stdscr, true);
            // Don't echo the users messages
            noecho();
            // Hide the cursor
            curs_set(0);

            refresh_view();
        });

        on<Trigger<CommandLineArguments>>().then([this](const CommandLineArguments& args) {
            if (args.size() == 2) {
                script_path = args[1];

                // Check if the script exists and load it if it does.
                if (utility::file::exists(script_path)) {
                    NUClear::log<NUClear::DEBUG>("Loading script: ", script_path, '\n');
                    load_script(script_path);
                    // Build our initial gui with context from loaded script
                    refresh_view();
                }
            }

            else {
                NUClear::log<NUClear::DEBUG>("Error: Expected 2 arguments on argv found ", args.size(), '\n');
                powerplant.shutdown();
            }
        });

        on<Trigger<LockServo>, With<RawSensors>>().then([this](const RawSensors& sensors) {
            auto id = selection < 2 ? 18 + selection : selection - 2;

            Frame::Target target;
            target.id       = id;
            target.position = utility::platform::get_raw_servo(target.id, sensors).present_position;
            target.gain     = default_gain;
            target.torque   = 100;

            script.frames[frame].targets.push_back(target);

            // Emit a waypoint so that the motor will go rigid at this angle
            auto waypoint      = std::make_unique<ServoTarget>();
            waypoint->time     = NUClear::clock::now();
            waypoint->id       = target.id;
            waypoint->gain     = target.gain;
            waypoint->position = target.position;
            waypoint->torque   = target.torque;
            emit(std::move(waypoint));
        });


        on<Always>().then([this] {
            switch (getch()) {
                case KEY_UP:  // Change selection up
                    selection = selection == 0 ? 19 : selection - 1;
                    break;
                case KEY_DOWN:  // Change selection down
                    selection = (selection + 1) % 20;
                    break;
                case 9:          // Swap between angle and gain
                case KEY_LEFT:   // Swap between angle and gain
                case KEY_RIGHT:  // Swap between angle and gain
                    angle_or_gain = !angle_or_gain;
                    break;
                case ',':  // Move left a frame
                    activate_frame(frame == 0 ? frame : frame - 1);
                    break;
                case '.':  // Move right a frame
                    activate_frame(frame == script.frames.size() - 1 ? frame : frame + 1);
                    break;
                case '\n':       // Edit selected field
                case KEY_ENTER:  // Edit selected field
                    edit_selection();
                    break;
                case ' ':  // Toggle lock mode
                    toggle_lock_motor();
                    break;
                case 'S':  // Save the current script
                    save_script();
                    break;
                case 'A':  // save script as
                    save_script_as();
                    break;
                case 'T':  // Edit this frames duration
                    edit_duration();
                    break;
                case 'N':  // New frame
                    new_frame();
                    break;
                case 'I':  // Delete frame
                    delete_frame();
                    break;
                case 'P':  // plays script through with correct durations
                    play_script();
                    break;
                case 'J':  // changes frame without robot moving
                    jump_to_frame();
                    break;
                case 'R':  // updates visual changes
                    refresh_view();
                    break;
                case 'M': mirror_script(); break;
                case 'G':  // allows multiple gains to be edited at once
                    edit_gain();
                    break;
                case ':':  // lists commands
                    help();
                    break;
                case 'X':  // shutdowns powerplant
                    powerplant.shutdown();
                    break;
            }
            // Update whatever visual changes we made
            refresh_view();
        });

        // When we shutdown end ncurses
        on<Shutdown>().then(endwin);
    }

    void ScriptTuner::activate_frame(int frame) {
        this->frame = frame;

        auto waypoints = std::make_unique<ServoTargets>();
        for (auto& target : script.frames[frame].targets) {
            waypoints->targets.emplace_back(NUClear::clock::now() + std::chrono::milliseconds(1000),
                                            target.id,
                                            target.position,
                                            target.gain,
                                            target.torque);
        }

        emit(std::move(waypoints));
    }

    void ScriptTuner::refresh_view() {
        // Clear our window
        erase();

        // Outer box
        box(stdscr, 0, 0);

        // Write our title
        attron(A_BOLD);
        mvprintw(0, (COLS - 14) / 2, " Script Tuner ");
        attroff(A_BOLD);

        // Top sections
        mvprintw(2, 2, "Script: %s", script_path.c_str());  // Output our scripts name
        mvprintw(3, 2, "Frames:");                          // The frames section is filled out after this
        mvprintw(4,
                 2,
                 "Duration: %ld",  // Output the selected frames duration
                 std::chrono::duration_cast<std::chrono::milliseconds>(script.frames[frame].duration).count());
        mvprintw(5, 2, "_");

        // Output all of our frame numbers and highlight the selected frame
        move(3, 10);
        for (size_t i = 0; i < script.frames.size(); ++i) {
            if (i == frame) {
                // Turn on highlighting to show this frame is selected
                attron(A_STANDOUT);
            }
            printw(std::to_string(i + 1).c_str());
            if (i == frame) {
                // Turn off highlighting
                attroff(A_STANDOUT);
            }
            printw(" ");
        }


        // Heading Commands
        attron(A_BOLD);
        mvprintw(LINES - 6, 2, "Commands");
        attroff(A_BOLD);
        mvprintw(LINES - 2, 2, "Type :help for a full list of commands");

        // Each Command
        const char* COMMANDS[] = {",", ".", "N", "I", " ", "T", "J", "G", "P", "S"};

        // Each Meaning
        const char* MEANINGS[] = {"Left a frame",
                                  "Right a frame",
                                  "New Frame",
                                  "Delete Frame",
                                  "Lock/Unlock",
                                  "Edit Duration",
                                  "Jump to Frame",
                                  "Change Gains",
                                  "Play",
                                  "Save"};

        // Prints commands and their meanings to the screen
        for (size_t i = 0; i < 10; i = i + 2) {
            attron(A_BOLD);
            attron(A_STANDOUT);
            mvprintw(LINES - 5, 2 + ((2 + 14) * (i / 2)), COMMANDS[i]);
            attroff(A_BOLD);
            attroff(A_STANDOUT);
            mvprintw(LINES - 5, 4 + ((2 + 14) * (i / 2)), MEANINGS[i]);
        }

        for (size_t i = 1; i < 10; i = i + 2) {
            attron(A_BOLD);
            attron(A_STANDOUT);
            mvprintw(LINES - 4, 2 + ((2 + 14) * ((i - 1) / 2)), COMMANDS[i]);
            attroff(A_BOLD);
            attroff(A_STANDOUT);
            mvprintw(LINES - 4, 4 + ((2 + 14) * ((i - 1) / 2)), MEANINGS[i]);
        }

        // Each motor
        const char* MOTOR_NAMES[] = {"Head Pan",
                                     "Head Tilt",
                                     "Right Shoulder Pitch",
                                     "Left  Shoulder Pitch",
                                     "Right Shoulder Roll",
                                     "Left  Shoulder Roll",
                                     "Right Elbow",
                                     "Left  Elbow",
                                     "Right Hip Yaw",
                                     "Left  Hip Yaw",
                                     "Right Hip Roll",
                                     "Left  Hip Roll",
                                     "Right Hip Pitch",
                                     "Left  Hip Pitch",
                                     "Right Knee",
                                     "Left  Knee",
                                     "Right Ankle Pitch",
                                     "Left  Ankle Pitch",
                                     "Right Ankle Roll",
                                     "Left  Ankle Roll"};

        // Loop through all our motors
        for (size_t i = 0; i < 20; ++i) {
            // Everything defaults to unlocked, we add locks as we find them
            mvprintw(i + 9, 2, "U");

            // Output the motor name
            attron(A_BOLD);
            mvprintw(i + 9, 4, MOTOR_NAMES[i]);
            attroff(A_BOLD);

            // Everything defaults to 0 angle and gain (unless we find one)
            mvprintw(i + 9, 26, "Angle:  -.--- Gain: ---.-");
        }

        for (auto& target : script.frames[frame].targets) {
            // Output that this frame is locked (we shuffle the head to the top of the list)
            mvprintw(((static_cast<uint32_t>(target.id) + 2) % 20) + 9, 2, "L");

            // Output this frames gain and angle
            mvprintw(((static_cast<uint32_t>(target.id) + 2) % 20) + 9,
                     26,
                     "Angle: %+.3f Gain: %5.1f",
                     target.position,
                     target.gain);
        }

        // Highlight our selected point
        mvchgat(selection + 9, angle_or_gain ? 26 : 40, angle_or_gain ? 13 : 11, A_STANDOUT, 0, nullptr);

        // We finished building
        refresh();
    }

    void ScriptTuner::toggle_lock_motor() {
        // This finds if we have this particular motor stored in the frame
        auto targetFinder = [=, this](const Frame::Target& target) {
            return (static_cast<uint32_t>(target.id) + 2) % 20 == selection;
        };

        // See if we have this target in our frame
        auto it = std::find_if(std::begin(script.frames[frame].targets),
                               std::end(script.frames[frame].targets),
                               targetFinder);

        // If we don't then save our current motor position as the position
        if (it == std::end(script.frames[frame].targets)) {

            emit<Scope::DIRECT>(std::make_unique<LockServo>());
        }
        else {
            // Remove this frame
            script.frames[frame].targets.erase(it);

            // Emit a waypoint so that the motor will turn off gain (go limp)
            auto waypoint      = std::make_unique<ServoTarget>();
            waypoint->time     = NUClear::clock::now();
            waypoint->id       = selection < 2 ? 18 + selection : selection - 2;
            waypoint->gain     = 0;
            waypoint->position = std::numeric_limits<float>::quiet_NaN();
            waypoint->torque   = 0;
            emit(std::move(waypoint));
        }
    }

    void ScriptTuner::new_frame() {
        // Make a new frame before our current with our current set of motor angles and unlocked/locked status
        auto new_frame = script.frames[frame];
        script.frames.insert(script.frames.begin() + frame, new_frame);
        script.frames[frame].duration = std::chrono::milliseconds(default_duration);
    }

    void ScriptTuner::delete_frame() {
        // Delete our current frame and go to the one before this one, if this is the last frame then ignore
        if (script.frames.size() > 1) {
            script.frames.erase(std::begin(script.frames) + frame);
            frame = frame < script.frames.size() ? frame : frame - 1;
        }
        else {
            script.frames.erase(std::begin(script.frames));
            script.frames.emplace_back();
            frame = 0;
        }
    }

    std::string ScriptTuner::user_input() {
        // Read characters until we see either esc or enter
        std::stringstream chars;

        // Keep reading until our termination case is reached
        while (true) {
            auto ch = getch();
            switch (ch) {
                case 27: return "";
                case '\n':
                case KEY_ENTER: return chars.str(); break;
                default:
                    chars << static_cast<char>(ch);
                    addch(ch);
                    break;
            }
        }
    }

    void ScriptTuner::load_script(const std::string& path) {
        // Load the YAML file
        YAML::Node node = YAML::LoadFile(path);

        // Decode the YAML node into a Script<BodySequence> object
        if (!YAML::convert<Script<BodySequence>>::decode(node, this->script)) {
            throw std::runtime_error("Failed to load script from " + path);
        }

        // Log a success message
        NUClear::log<NUClear::DEBUG>("Successfully loaded script from:", path);
    }

    void ScriptTuner::save_script() {
        YAML::Node n(script);
        utility::file::writeToFile(script_path, n);
    }

    void ScriptTuner::edit_duration() {
        // Move to the correct position and erase the old duration
        move(4, 12);
        for (int i = 0; i < 10; ++i) {
            addch(' ');
        }
        move(4, 12);

        // Get the users input
        std::string result = user_input();

        // If we have a result
        if (!result.empty()) {
            try {
                int num                       = stoi(result);
                script.frames[frame].duration = std::chrono::milliseconds(num);
            }
            // If it's not a number then ignore and beep
            catch (std::invalid_argument&) {
                beep();
            }
        }
    }

    void ScriptTuner::edit_selection() {
        // Erase our old text
        mvprintw(selection + 9, angle_or_gain ? 33 : 46, " ");

        // Move to our point
        move(selection + 9, angle_or_gain ? 33 : 46);

        // Get the users input
        std::string result = user_input();

        // If we have a result
        if (!result.empty()) {
            try {
                double num = stod(result);

                // This finds if we have this particular motor stored in the frame
                auto targetFinder = [=, this](const Frame::Target& target) {
                    return (static_cast<uint32_t>(target.id) + 2) % 20 == selection;
                };

                // See if we have this target in our frame
                auto it = std::find_if(std::begin(script.frames[frame].targets),
                                       std::end(script.frames[frame].targets),
                                       targetFinder);

                // If we don't have this frame
                if (it == std::end(script.frames[frame].targets)) {
                    it           = script.frames[frame].targets.emplace(std::end(script.frames[frame].targets));
                    auto id      = selection < 2 ? 18 + selection : selection - 2;
                    it->id       = id;
                    it->position = 0;
                    it->gain     = default_gain;
                }

                // If we are entering an angle
                if (angle_or_gain) {

                    // Normalize our angle to be between -pi and pi
                    num = utility::math::angle::normalizeAngle(num);

                    it->position = num;
                    // Convert our angle to be between -pi and pi
                }
                // If it is a gain
                else {
                    if (num >= 0 && num <= 100) {
                        it->gain = num;
                    }
                    else {
                        beep();
                    }
                    // Check if the value is < 0 or > 100
                }
            }
            // If it's not a number then ignore and beep
            catch (std::invalid_argument&) {
                beep();
            }
        }
    }


    void ScriptTuner::help() {
        move(LINES - 6, 12);
        curs_set(1);
        std::string tempcommand = user_input();

        if (tempcommand == "help") {
            curs_set(0);

            const char* ALL_COMMANDS[] =
                {",", ".", "N", "I", " ", "T", "J", "G", "P", "S", "A", "R", "M", "X", "Ctr C"};

            const char* ALL_MEANINGS[] = {"Left a frame",
                                          "Right a frame",
                                          "New Frame",
                                          "Delete Frame",
                                          "Lock/Unlock",
                                          "Edit Duration",
                                          "Jump to Frame",
                                          "Edit the gains of an entire Script or Frame",
                                          "Play",
                                          "Save",
                                          "Saves Script As)",
                                          "Manual Refresh View",
                                          "Mirrors the script",
                                          "Exit (this works to exit help and edit_gain)",
                                          "Quit Scripttuner"};

            size_t longestCommand = 0;
            for (const auto& command : ALL_COMMANDS) {
                longestCommand = std::max(longestCommand, std::strlen(command));
            }

            erase();
            box(stdscr, 0, 0);
            attron(A_BOLD);
            mvprintw(0, (COLS - 14) / 2, " Script Tuner ");
            mvprintw(3, 2, "Help Commands:");
            attroff(A_BOLD);
            for (size_t i = 0; i < 15; i++) {
                mvprintw(5 + i, 2, ALL_COMMANDS[i]);
                mvprintw(5 + i, longestCommand + 4, ALL_MEANINGS[i]);
            }

            while (getch() != 'X') {
                erase();
                box(stdscr, 0, 0);
                attron(A_BOLD);
                mvprintw(0, (COLS - 14) / 2, " Script Tuner ");
                mvprintw(3, 2, "Help Commands:");
                attroff(A_BOLD);

                for (size_t i = 0; i < 15; i++) {
                    mvprintw(5 + i, 2, ALL_COMMANDS[i]);
                    mvprintw(5 + i, longestCommand + 4, ALL_MEANINGS[i]);
                }
            }
            refresh_view();
        }
        else {
            refresh_view();
        }
        curs_set(0);
    }

    void ScriptTuner::play_script() {
        emit<Task>(utility::skill::load_script<BodySequence>(script));
    }

    void ScriptTuner::jump_to_frame() {
        mvprintw(5, 2, "Jump To Frame:");
        move(5, 17);
        curs_set(1);
        user_input_to_frame();
        curs_set(0);
    }

    void ScriptTuner::mirror_script() {
        for (auto& f : script.frames) {

            Frame new_frame;
            new_frame.duration = f.duration;

            for (auto& target : f.targets) {

                switch (target.id.value) {
                    case ServoID::HEAD_YAW:
                        new_frame.targets.emplace_back(ServoID::HEAD_YAW, target.position, target.gain, target.torque);
                        break;
                    case ServoID::HEAD_PITCH:
                        new_frame.targets.emplace_back(ServoID::HEAD_PITCH,
                                                       target.position,
                                                       target.gain,
                                                       target.torque);
                        break;
                    case ServoID::R_SHOULDER_PITCH:
                        new_frame.targets.emplace_back(ServoID::L_SHOULDER_PITCH,
                                                       target.position,
                                                       target.gain,
                                                       target.torque);
                        break;
                    case ServoID::L_SHOULDER_PITCH:
                        new_frame.targets.emplace_back(ServoID::R_SHOULDER_PITCH,
                                                       target.position,
                                                       target.gain,
                                                       target.torque);
                        break;
                    case ServoID::R_ELBOW:
                        new_frame.targets.emplace_back(ServoID::L_ELBOW, target.position, target.gain, target.torque);
                        break;
                    case ServoID::L_ELBOW:
                        new_frame.targets.emplace_back(ServoID::R_ELBOW, target.position, target.gain, target.torque);
                        break;
                    case ServoID::R_HIP_PITCH:
                        new_frame.targets.emplace_back(ServoID::L_HIP_PITCH,
                                                       target.position,
                                                       target.gain,
                                                       target.torque);
                        break;
                    case ServoID::L_HIP_PITCH:
                        new_frame.targets.emplace_back(ServoID::R_HIP_PITCH,
                                                       target.position,
                                                       target.gain,
                                                       target.torque);
                        break;
                    case ServoID::R_KNEE:
                        new_frame.targets.emplace_back(ServoID::L_KNEE, target.position, target.gain, target.torque);
                        break;
                    case ServoID::L_KNEE:
                        new_frame.targets.emplace_back(ServoID::R_KNEE, target.position, target.gain, target.torque);
                        break;
                    case ServoID::R_ANKLE_PITCH:
                        new_frame.targets.emplace_back(ServoID::L_ANKLE_PITCH,
                                                       target.position,
                                                       target.gain,
                                                       target.torque);
                        break;
                    case ServoID::L_ANKLE_PITCH:
                        new_frame.targets.emplace_back(ServoID::R_ANKLE_PITCH,
                                                       target.position,
                                                       target.gain,
                                                       target.torque);
                        break;
                    case ServoID::R_SHOULDER_ROLL:
                        new_frame.targets.emplace_back(ServoID::L_SHOULDER_ROLL,
                                                       -target.position,
                                                       target.gain,
                                                       target.torque);
                        break;
                    case ServoID::L_SHOULDER_ROLL:
                        new_frame.targets.emplace_back(ServoID::R_SHOULDER_ROLL,
                                                       -target.position,
                                                       target.gain,
                                                       target.torque);
                        break;
                    case ServoID::R_HIP_ROLL:
                        new_frame.targets.emplace_back(ServoID::L_HIP_ROLL,
                                                       -target.position,
                                                       target.gain,
                                                       target.torque);
                        break;
                    case ServoID::L_HIP_ROLL:
                        new_frame.targets.emplace_back(ServoID::R_HIP_ROLL,
                                                       -target.position,
                                                       target.gain,
                                                       target.torque);
                        break;
                    case ServoID::R_ANKLE_ROLL:
                        new_frame.targets.emplace_back(ServoID::L_ANKLE_ROLL,
                                                       -target.position,
                                                       target.gain,
                                                       target.torque);
                        break;
                    case ServoID::L_ANKLE_ROLL:
                        new_frame.targets.emplace_back(ServoID::R_ANKLE_ROLL,
                                                       -target.position,
                                                       target.gain,
                                                       target.torque);
                        break;
                    case ServoID::R_HIP_YAW:
                        new_frame.targets.emplace_back(ServoID::L_HIP_YAW,
                                                       -target.position,
                                                       target.gain,
                                                       target.torque);
                        break;
                    case ServoID::L_HIP_YAW:
                        new_frame.targets.emplace_back(ServoID::R_HIP_YAW,
                                                       -target.position,
                                                       target.gain,
                                                       target.torque);
                        break;
                    case ServoID::NUMBER_OF_SERVOS:
                    default: break;
                }  // end switch(target.id)
            }
            f = new_frame;
            refresh_view();
        }
    }

    void ScriptTuner::save_script_as() {
        move(5, 2);
        curs_set(1);
        std::string save_script_as = user_input();
        if (utility::file::exists(save_script_as)) {
            bool print = true;
            while (print) {
                mvprintw(6, 2, "This file already exists.");
                mvprintw(7, 2, "Press Enter to overwrite, or X to return to script.");
                switch (getch()) {
                    case '\n':
                    case KEY_ENTER:
                        move(5, 2);
                        curs_set(0);
                        print       = false;
                        script_path = save_script_as;
                        save_script();
                        refresh_view();
                        break;
                    case 'X':
                        move(5, 2);
                        curs_set(0);
                        print = false;
                        refresh_view();
                        break;
                }
            }
        }
        else {
            script_path = save_script_as;
            save_script();
            move(5, 2);
            curs_set(0);
            refresh_view();
        }
    }

    void ScriptTuner::edit_gain() {
        erase();
        box(stdscr, 0, 0);
        attron(A_BOLD);
        mvprintw(0, (COLS - 14) / 2, " Script Tuner ");
        mvprintw(3, 2, "Edit Gain");
        attroff(A_BOLD);
        mvprintw(5, 2, "For Entire Script:");
        mvprintw(6, 2, "All: ---.- Upper: ---.- Lower: ---.-");
        mvprintw(7, 2, "For Frame: %ld", frame + 1);
        mvprintw(8, 2, "All: ---.- Upper: ---.- Lower: ---.-");
        mvprintw(10, 2, "Use X to exit Edit Gain");
        move(6, 7);
        curs_set(0);
        size_t YPOSITION[3][3] = {{6, 6, 6}, {7, 0, 0}, {8, 8, 8}};
        size_t XPOSITION[3][3] = {{7, 20, 33}, {12, 0, 0}, {7, 20, 33}};
        size_t i               = 0;
        size_t j               = 0;
        float upperGainS       = -1;
        float lowerGainS       = -1;
        float upperGainF       = -1;
        float lowerGainF       = -1;
        bool editScript        = false;
        bool editFrame         = false;
        bool changedUpper      = false;
        bool changedLower      = false;
        bool edit_gainRun      = true;
        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, A_STANDOUT, 0, nullptr);

        while (edit_gainRun) {

            switch (getch()) {
                case 'X': edit_gainRun = false; break;
                case KEY_UP:
                    if (YPOSITION[i][j] == 0 && XPOSITION[i][j] == 0) {
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, 0, 0, nullptr);
                        i = ((i - 2) + 3) % 3;
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, A_STANDOUT, 0, nullptr);
                    }
                    else if (YPOSITION[i][j] == 7 && XPOSITION[i][j] == 12) {
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, 0, 0, nullptr);
                        i = ((i - 1) + 3) % 3;
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, A_STANDOUT, 0, nullptr);
                    }
                    else if (YPOSITION[i][j] == 8 && (XPOSITION[i][j] == 20 || XPOSITION[i][j] == 33)) {
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, 0, 0, nullptr);
                        i = ((i - 2) + 3) % 3;
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, A_STANDOUT, 0, nullptr);
                    }
                    else {
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, 0, 0, nullptr);
                        i = ((i - 1) + 3) % 3;
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, A_STANDOUT, 0, nullptr);
                    }
                    break;
                case KEY_DOWN:
                    if (YPOSITION[i][j] == 0 && XPOSITION[i][j] == 0) {
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, 0, 0, nullptr);
                        i = (i + 2) % 3;
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, A_STANDOUT, 0, nullptr);
                    }
                    else if (YPOSITION[i][j] == 7 && XPOSITION[i][j] == 12) {
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, 0, 0, nullptr);
                        i = (i + 1) % 3;
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, A_STANDOUT, 0, nullptr);
                    }
                    else if (YPOSITION[i][j] == 6 && (XPOSITION[i][j] == 20 || XPOSITION[i][j] == 33)) {
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, 0, 0, nullptr);
                        i = (i + 2) % 3;
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, A_STANDOUT, 0, nullptr);
                    }
                    else {
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, 0, 0, nullptr);
                        i = (i + 1) % 3;
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, A_STANDOUT, 0, nullptr);
                    }
                    break;
                case KEY_LEFT:
                    if (YPOSITION[i][j] == 0 && XPOSITION[i][j] == 0) {
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, 0, 0, nullptr);
                        j = 0;
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, A_STANDOUT, 0, nullptr);
                    }
                    else if (YPOSITION[i][j] == 7 && XPOSITION[i][j] == 12) {
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, 0, 0, nullptr);
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, A_STANDOUT, 0, nullptr);
                    }
                    else {
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, 0, 0, nullptr);
                        j = (j - 1) % 3;
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, A_STANDOUT, 0, nullptr);
                    }
                    break;
                case KEY_RIGHT:
                    if (YPOSITION[i][j] == 0 && XPOSITION[i][j] == 0) {
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, 0, 0, nullptr);
                        j = 0;
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, A_STANDOUT, 0, nullptr);
                    }
                    else if (YPOSITION[i][j] == 7 && XPOSITION[i][j] == 12) {
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, 0, 0, nullptr);
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, A_STANDOUT, 0, nullptr);
                    }
                    else {
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, 0, 0, nullptr);
                        j = (j + 1) % 3;
                        mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, A_STANDOUT, 0, nullptr);
                    }
                    break;
                case '\n':
                case KEY_ENTER:
                    float newGain = 0;
                    // tracks editing

                    if (YPOSITION[i][j] == 6) {
                        editScript = true;
                    }
                    else if (YPOSITION[i][j] == 8) {
                        editFrame = true;
                    }

                    // prints user input to screen
                    if (YPOSITION[i][j] == 7 && XPOSITION[i][j] == 12) {
                        mvprintw(YPOSITION[i][j], XPOSITION[i][j], "     ");
                        move(YPOSITION[i][j], XPOSITION[i][j]);
                        user_input_to_frame();
                        mvprintw(YPOSITION[i][j], XPOSITION[i][j], "%ld", frame + 1);
                    }
                    else {
                        mvprintw(YPOSITION[i][j], XPOSITION[i][j], "     ");
                        move(YPOSITION[i][j], XPOSITION[i][j]);
                        newGain = user_input_to_gain();
                        if (std::isnan(newGain)) {
                            mvprintw(YPOSITION[i][j], XPOSITION[i][j], "---.-");
                            upperGainS = -1;
                            lowerGainS = -1;
                            upperGainF = -1;
                            lowerGainF = -1;
                        }
                        else {

                            mvprintw(YPOSITION[i][j], XPOSITION[i][j], "%5.1f", newGain);

                            // allows separate gains for upper and lower motors
                            if (XPOSITION[i][j] == 20) {
                                if (YPOSITION[i][j] == 6) {
                                    upperGainS = newGain;
                                }
                                else {
                                    upperGainF = newGain;
                                }

                                // Zero out the "ALL" option
                                if (YPOSITION[i][j] == 6) {
                                    mvprintw(6, 7, "---.-");
                                }
                                else {
                                    mvprintw(8, 7, "---.-");
                                }
                                changedUpper = true;
                            }
                            else if (XPOSITION[i][j] == 33) {
                                if (YPOSITION[i][j] == 6) {
                                    lowerGainS = newGain;
                                }
                                else {
                                    lowerGainF = newGain;
                                }

                                // Zero out the both option
                                if (YPOSITION[i][j] == 6) {
                                    mvprintw(6, 7, "---.-");
                                }
                                else {
                                    mvprintw(8, 7, "---.-");
                                }
                                changedLower = true;
                            }
                            else {

                                // Set upper and lower
                                if (XPOSITION[i][j] == 7) {
                                    if (YPOSITION[i][j] == 6) {
                                        upperGainS = newGain;
                                        lowerGainS = newGain;
                                        mvprintw(6, 7, "%5.1f", upperGainS);
                                        mvprintw(6, 20, "---.-");
                                        mvprintw(6, 33, "---.-");
                                    }
                                    else {
                                        upperGainF = newGain;
                                        lowerGainF = newGain;
                                        mvprintw(8, 7, "%5.1f", upperGainF);
                                        mvprintw(8, 20, "---.-");
                                        mvprintw(8, 33, "---.-");
                                    }
                                }
                                changedUpper = true;
                                changedLower = true;
                            }
                            /*
                            mvprintw(20,2,"upperGainS = %5.1f",upperGainS);
                            mvprintw(21,2,"lowerGainS = %5.1f",lowerGainS);
                            mvprintw(22,2,"upperGainF = %5.1f",upperGainF);
                            mvprintw(23,2,"lowerGainF = %5.1f",lowerGainF);
                            */

                            // if user has entered the same gain in upper and lower then automatically prints
                            // value in both and dashes upper and lower
                            if ((upperGainS == lowerGainS) && (upperGainS >= 0)) {
                                mvprintw(6, 7, "%5.1f", upperGainS);
                                mvprintw(6, 20, "---.-");
                                mvprintw(6, 33, "---.-");
                            }
                            if ((upperGainF == lowerGainF) && (upperGainF >= 0)) {
                                mvprintw(8, 7, "%5.1f", upperGainF);
                                mvprintw(8, 20, "---.-");
                                mvprintw(8, 33, "---.-");
                            }
                        }
                    }  // end KEY_ENTER else
                    mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, A_STANDOUT, 0, nullptr);
                    break;  // end case KEY_ENTER
            }  // switch

        }  // while

        // loop through all frames in script and edit gains
        if (editScript) {
            std::cout << "Hello!" << std::endl;
            for (auto& f : script.frames) {
                for (auto& target : f.targets) {
                    switch (target.id.value) {
                        case ServoID::HEAD_YAW:
                        case ServoID::HEAD_PITCH:
                        case ServoID::R_SHOULDER_PITCH:
                        case ServoID::L_SHOULDER_PITCH:
                        case ServoID::R_SHOULDER_ROLL:
                        case ServoID::L_SHOULDER_ROLL:
                        case ServoID::R_ELBOW:
                        case ServoID::L_ELBOW:
                            if (changedUpper && (upperGainS >= 0)) {
                                target.gain = upperGainS;
                            }
                            break;
                        case ServoID::R_HIP_YAW:
                        case ServoID::L_HIP_YAW:
                        case ServoID::R_HIP_ROLL:
                        case ServoID::L_HIP_ROLL:
                        case ServoID::R_HIP_PITCH:
                        case ServoID::L_HIP_PITCH:
                        case ServoID::R_KNEE:
                        case ServoID::L_KNEE:
                        case ServoID::R_ANKLE_PITCH:
                        case ServoID::L_ANKLE_PITCH:
                        case ServoID::R_ANKLE_ROLL:
                        case ServoID::L_ANKLE_ROLL:
                            if (changedLower && (lowerGainS >= 0)) {
                                target.gain = lowerGainS;
                            }
                            break;
                        case ServoID::NUMBER_OF_SERVOS:
                        default: break;
                    }
                }
            }
        }

        // edit gains for only specifc frame
        if (editFrame) {
            for (auto& target : script.frames[frame].targets) {
                switch (target.id.value) {
                    case ServoID::HEAD_YAW:
                    case ServoID::HEAD_PITCH:
                    case ServoID::R_SHOULDER_PITCH:
                    case ServoID::L_SHOULDER_PITCH:
                    case ServoID::R_SHOULDER_ROLL:
                    case ServoID::L_SHOULDER_ROLL:
                    case ServoID::R_ELBOW:
                    case ServoID::L_ELBOW:
                        if (changedUpper && (upperGainF >= 0)) {
                            target.gain = upperGainF;
                        }
                        break;
                    case ServoID::R_HIP_YAW:
                    case ServoID::L_HIP_YAW:
                    case ServoID::R_HIP_ROLL:
                    case ServoID::L_HIP_ROLL:
                    case ServoID::R_HIP_PITCH:
                    case ServoID::L_HIP_PITCH:
                    case ServoID::R_KNEE:
                    case ServoID::L_KNEE:
                    case ServoID::R_ANKLE_PITCH:
                    case ServoID::L_ANKLE_PITCH:
                    case ServoID::R_ANKLE_ROLL:
                    case ServoID::L_ANKLE_ROLL:
                        if (changedLower && (lowerGainF >= 0)) {
                            target.gain = lowerGainF;
                        }
                        break;
                    case ServoID::NUMBER_OF_SERVOS:
                    default: break;
                }
            }
        }
        refresh_view();
    }

    void ScriptTuner::user_input_to_frame() {
        std::string tempframe = user_input();
        if (!tempframe.empty() && tempframe.size() <= 4) {
            try {
                int tempframe2 = stoi(tempframe);

                // makes tempframe2 always positive
                if (tempframe2 <= 0) {
                    tempframe2 = -1 * tempframe2;
                }
                else {
                    tempframe2 = tempframe2;
                }

                // checks user input is within correct range
                if (static_cast<size_t>(tempframe2) <= script.frames.size()) {

                    frame = tempframe2 - 1;
                }
                else {
                    beep();
                }
            }
            catch (std::invalid_argument&) {
                beep();
            }
        }
    }

    float ScriptTuner::user_input_to_gain() {
        std::string tempGain = user_input();
        try {
            if (!tempGain.empty()) {
                float tempGain2 = stof(tempGain);
                if (tempGain2 >= 0 && tempGain2 <= 100) {
                    return tempGain2;
                }
                beep();
            }
        }
        catch (std::invalid_argument&) {
            beep();
        }

        return std::numeric_limits<float>::quiet_NaN();
    }
}  // namespace module::purpose
