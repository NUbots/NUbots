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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "ScriptTuner.h"

#include "message/motion/ServoTarget.h"
#include "message/platform/darwin/DarwinSensors.h"

#include "utility/behaviour/Action.h"
#include "utility/file/fileutil.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/math/angle.h"
#include "utility/platform/darwin/DarwinSensors.h"

#include <ncurses.h>
#include <cstdio>
#include <sstream>

namespace module {
namespace behaviour {
    namespace tools {

        using NUClear::message::CommandLineArguments;

        using extension::ExecuteScript;
        using extension::Script;

        using message::motion::ServoTarget;
        using message::platform::darwin::DarwinSensors;

        using utility::behaviour::RegisterAction;
        using LimbID  = utility::input::LimbID;
        using ServoID = utility::input::ServoID;

        struct LockServo {};

        ScriptTuner::ScriptTuner(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , id(size_t(this) * size_t(this) - size_t(this))
            , scriptPath("Initializing...")
            , script()
            , frame(0)
            , selection(0)
            , angleOrGain(true)
            , running(true) {

            // Add a blank frame to start with
            script.frames.emplace_back();
            script.frames.back().duration = std::chrono::milliseconds(defaultDuration);

            on<Trigger<CommandLineArguments>>().then([this](const CommandLineArguments& args) {
                if (args.size() == 2) {
                    scriptPath = args[1];

                    // Check if the script exists and load it if it does.
                    if (utility::file::exists(scriptPath)) {
                        NUClear::log<NUClear::DEBUG>("Loading script: ", scriptPath, '\n');
                        loadScript(scriptPath);
                        // Build our initial gui with context from loaded script
                        refreshView();
                    }
                }

                else {
                    NUClear::log<NUClear::DEBUG>("Error: Expected 2 arguments on argv found ", args.size(), '\n');
                    powerplant.shutdown();
                }
            });

            on<Trigger<LockServo>, With<DarwinSensors>>().then([this](const DarwinSensors& sensors) {
                auto id = selection < 2 ? 18 + selection : selection - 2;

                Script::Frame::Target target;

                target.id       = id;
                target.position = utility::platform::darwin::getDarwinServo(target.id, sensors).presentPosition;
                target.gain     = defaultGain;
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

            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction{
                id,
                "Script Tuner",
                {std::pair<float, std::set<LimbID>>(
                    1, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG, LimbID::LEFT_ARM, LimbID::RIGHT_ARM, LimbID::HEAD})},
                [this](const std::set<LimbID>&) {},
                [this](const std::set<LimbID>&) {},
                [this](const std::set<ServoID>&) {}}));

            // Start curses mode
            initscr();
            // Capture our characters immediately (but pass through signals)
            cbreak();
            // Capture arrows and function keys
            keypad(stdscr, true);
            // Don't echo the users messages
            noecho();
            // Hide the cursor
            curs_set(false);

            // Trigger when stdin has something to read
            on<IO>(STDIN_FILENO, IO::READ).then([this] {
                // Get the character the user has typed
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
                        angleOrGain = !angleOrGain;
                        break;
                    case ',':  // Move left a frame
                        activateFrame(frame == 0 ? frame : frame - 1);
                        break;
                    case '.':  // Move right a frame
                        activateFrame(frame == script.frames.size() - 1 ? frame : frame + 1);
                        break;
                    case '\n':       // Edit selected field
                    case KEY_ENTER:  // Edit selected field
                        editSelection();
                        break;
                    case ' ':  // Toggle lock mode
                        toggleLockMotor();
                        break;
                    case 'S':  // Save the current script
                        saveScript();
                        break;
                    case 'A':  // save script as
                        saveScriptAs();
                        break;
                    case 'T':  // Edit this frames duration
                        editDuration();
                        break;
                    case 'N':  // New frame
                        newFrame();
                        break;
                    case 'I':  // Delete frame
                        deleteFrame();
                        break;
                    case 'P':  // plays script through with correct durations
                        playScript();
                        break;
                    case 'J':  // changes frame without out robot moving
                        jumpToFrame();
                        break;
                    case 'R':  // updates visual changes
                        refreshView();
                        break;
                    case 'M': mirrorScript(); break;
                    case 'G':  // allows multiple gains to be edited at once
                        editGain();
                        break;
                    case ':':  // lists commands
                        help();
                        break;
                    case 'X':  // shutdowns powerplant
                        powerplant.shutdown();
                        break;
                }

                // Update whatever visual changes we made
                refreshView();
            });

            // When we shutdown end ncurses
            on<Shutdown>().then(endwin);
        }

        void ScriptTuner::activateFrame(int frame) {
            this->frame = frame;

            auto waypoints = std::make_unique<std::vector<ServoTarget>>();
            for (auto& target : script.frames[frame].targets) {
                waypoints->push_back(ServoTarget{NUClear::clock::now() + std::chrono::milliseconds(1000),
                                                 target.id,
                                                 target.position,
                                                 target.gain,
                                                 target.torque});
            }

            emit(std::move(waypoints));
        }

        void ScriptTuner::refreshView() {

            // Clear our window
            erase();

            // Outer box
            box(stdscr, 0, 0);

            // Write our title
            attron(A_BOLD);
            mvprintw(0, (COLS - 14) / 2, " Script Tuner ");
            attroff(A_BOLD);

            // Top sections
            mvprintw(2, 2, "Script: %s", scriptPath.c_str());  // Output our scripts name
            mvprintw(3, 2, "Frames:");                         // The frames section is filled out after this
            mvprintw(4,
                     2,
                     "Duration: %d",  // Output the selected frames duration
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
                                         "Left Shoulder Pitch",
                                         "Right Shoulder Roll",
                                         "Left Shoulder Roll",
                                         "Right Elbow",
                                         "Left Elbow",
                                         "Right Hip Yaw",
                                         "Left Hip Yaw",
                                         "Right Hip Roll",
                                         "Left Hip Roll",
                                         "Right Hip Pitch",
                                         "Left Hip Pitch",
                                         "Right Knee",
                                         "Left Knee",
                                         "Right Ankle Pitch",
                                         "Left Ankle Pitch",
                                         "Right Ankle Roll",
                                         "Left Ankle Roll"};

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
            mvchgat(selection + 9, angleOrGain ? 26 : 40, angleOrGain ? 13 : 11, A_STANDOUT, 0, nullptr);

            // We finished building
            refresh();
        }

        void ScriptTuner::toggleLockMotor() {

            // This finds if we have this particular motor stored in the frame
            auto targetFinder = [=](const Script::Frame::Target& target) {
                return (static_cast<uint32_t>(target.id) + 2) % 20 == selection;
            };

            // See if we have this target in our frame
            auto it = std::find_if(
                std::begin(script.frames[frame].targets), std::end(script.frames[frame].targets), targetFinder);

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

        void ScriptTuner::newFrame() {
            // Make a new frame before our current with our current set of motor angles and unlocked/locked status
            auto newFrame = script.frames[frame];
            script.frames.insert(script.frames.begin() + frame, newFrame);
            script.frames[frame].duration = std::chrono::milliseconds(defaultDuration);
        }

        void ScriptTuner::deleteFrame() {
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

        std::string ScriptTuner::userInput() {
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

        void ScriptTuner::loadScript(const std::string& path) {
            script = YAML::LoadFile(path).as<Script>();
        }

        void ScriptTuner::saveScript() {
            YAML::Node n(script);
            utility::file::writeToFile(scriptPath, n);
        }

        void ScriptTuner::editDuration() {

            // Move to the correct position and erase the old duration
            move(4, 12);
            for (int i = 0; i < 10; ++i) {
                addch(' ');
            }
            move(4, 12);

            // Get the users input
            std::string result = userInput();

            // If we have a result
            if (!result.empty()) {
                try {
                    int num                       = stoi(result);
                    script.frames[frame].duration = std::chrono::milliseconds(num);
                }
                // If it's not a number then ignore and beep
                catch (std::invalid_argument) {
                    beep();
                }
            }
        }

        void ScriptTuner::editSelection() {

            // Erase our old text
            mvprintw(selection + 9, angleOrGain ? 33 : 46, " ");

            // Move to our point
            move(selection + 9, angleOrGain ? 33 : 46);

            // Get the users input
            std::string result = userInput();

            // If we have a result
            if (!result.empty()) {
                try {
                    double num = stod(result);

                    // This finds if we have this particular motor stored in the frame
                    auto targetFinder = [=](const Script::Frame::Target& target) {
                        return (static_cast<uint32_t>(target.id) + 2) % 20 == selection;
                    };

                    // See if we have this target in our frame
                    auto it = std::find_if(
                        std::begin(script.frames[frame].targets), std::end(script.frames[frame].targets), targetFinder);

                    // If we don't have this frame
                    if (it == std::end(script.frames[frame].targets)) {
                        it           = script.frames[frame].targets.emplace(std::end(script.frames[frame].targets));
                        auto id      = selection < 2 ? 18 + selection : selection - 2;
                        it->id       = id;
                        it->position = 0;
                        it->gain     = defaultGain;
                    }

                    // If we are entering an angle
                    if (angleOrGain) {

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
                catch (std::invalid_argument) {
                    beep();
                }
            }
        }


        void ScriptTuner::help() {

            move(LINES - 6, 12);
            curs_set(true);
            std::string tempcommand = userInput();

            if (tempcommand.compare("help") == 0) {
                curs_set(false);

                const char* ALL_COMMANDS[] = {
                    ",", ".", "N", "I", " ", "T", "J", "G", "P", "S", "A", "R", "M", "X", "Ctr C"};

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
                                              "Exit (this works to exit help and editGain)",
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
                refreshView();
            }
            else {
                refreshView();
            }
            curs_set(false);
        }

        // emits a message so motion can pick up the script
        void ScriptTuner::playScript() {
            emit(std::make_unique<ExecuteScript>(id, script, NUClear::clock::now()));
        }

        // allows user to jump to a specific frame without engaging the motors
        void ScriptTuner::jumpToFrame() {
            mvprintw(5, 2, "Jump To Frame:");
            move(5, 17);
            curs_set(true);
            userInputToFrame();
            curs_set(false);
        }

        // switches angle and gains between corresponding left and right motors, flips script around z axis
        void ScriptTuner::mirrorScript() {

            for (auto& f : script.frames) {

                Script::Frame newFrame;
                newFrame.duration = f.duration;

                for (auto& target : f.targets) {

                    switch (target.id.value) {
                        case ServoID::HEAD_YAW:
                            newFrame.targets.push_back(
                                {ServoID::HEAD_YAW, target.position, target.gain, target.torque});
                            break;
                        case ServoID::HEAD_PITCH:
                            newFrame.targets.push_back(
                                {ServoID::HEAD_PITCH, target.position, target.gain, target.torque});
                            break;
                        case ServoID::R_SHOULDER_PITCH:
                            newFrame.targets.push_back(
                                {ServoID::L_SHOULDER_PITCH, target.position, target.gain, target.torque});
                            break;
                        case ServoID::L_SHOULDER_PITCH:
                            newFrame.targets.push_back(
                                {ServoID::R_SHOULDER_PITCH, target.position, target.gain, target.torque});
                            break;
                        case ServoID::R_ELBOW:
                            newFrame.targets.push_back({ServoID::L_ELBOW, target.position, target.gain, target.torque});
                            break;
                        case ServoID::L_ELBOW:
                            newFrame.targets.push_back({ServoID::R_ELBOW, target.position, target.gain, target.torque});
                            break;
                        case ServoID::R_HIP_PITCH:
                            newFrame.targets.push_back(
                                {ServoID::L_HIP_PITCH, target.position, target.gain, target.torque});
                            break;
                        case ServoID::L_HIP_PITCH:
                            newFrame.targets.push_back(
                                {ServoID::R_HIP_PITCH, target.position, target.gain, target.torque});
                            break;
                        case ServoID::R_KNEE:
                            newFrame.targets.push_back({ServoID::L_KNEE, target.position, target.gain, target.torque});
                            break;
                        case ServoID::L_KNEE:
                            newFrame.targets.push_back({ServoID::R_KNEE, target.position, target.gain, target.torque});
                            break;
                        case ServoID::R_ANKLE_PITCH:
                            newFrame.targets.push_back(
                                {ServoID::L_ANKLE_PITCH, target.position, target.gain, target.torque});
                            break;
                        case ServoID::L_ANKLE_PITCH:
                            newFrame.targets.push_back(
                                {ServoID::R_ANKLE_PITCH, target.position, target.gain, target.torque});
                            break;
                        case ServoID::R_SHOULDER_ROLL:
                            newFrame.targets.push_back(
                                {ServoID::L_SHOULDER_ROLL, -target.position, target.gain, target.torque});
                            break;
                        case ServoID::L_SHOULDER_ROLL:
                            newFrame.targets.push_back(
                                {ServoID::R_SHOULDER_ROLL, -target.position, target.gain, target.torque});
                            break;
                        case ServoID::R_HIP_ROLL:
                            newFrame.targets.push_back(
                                {ServoID::L_HIP_ROLL, -target.position, target.gain, target.torque});
                            break;
                        case ServoID::L_HIP_ROLL:
                            newFrame.targets.push_back(
                                {ServoID::R_HIP_ROLL, -target.position, target.gain, target.torque});
                            break;
                        case ServoID::R_ANKLE_ROLL:
                            newFrame.targets.push_back(
                                {ServoID::L_ANKLE_ROLL, -target.position, target.gain, target.torque});
                            break;
                        case ServoID::L_ANKLE_ROLL:
                            newFrame.targets.push_back(
                                {ServoID::R_ANKLE_ROLL, -target.position, target.gain, target.torque});
                            break;
                        case ServoID::R_HIP_YAW:
                            newFrame.targets.push_back(
                                {ServoID::L_HIP_YAW, -target.position, target.gain, target.torque});
                            break;
                        case ServoID::L_HIP_YAW:
                            newFrame.targets.push_back(
                                {ServoID::R_HIP_YAW, -target.position, target.gain, target.torque});
                            break;
                        case ServoID::NUMBER_OF_SERVOS:
                        default: break;
                    }  // end switch(target.id)
                }
                f = newFrame;
                refreshView();
            }
        }  // end mirrorScript()

        // change scriptPath and then call saveScript to Save As
        void ScriptTuner::saveScriptAs() {
            move(5, 2);
            curs_set(true);
            std::string saveScriptAs = userInput();
            if (utility::file::exists(saveScriptAs)) {
                bool print = true;
                while (print) {
                    mvprintw(6, 2, "This file already exists.");
                    mvprintw(7, 2, "Press Enter to overwrite, or X to return to script.");
                    switch (getch()) {
                        case '\n':
                        case KEY_ENTER:
                            move(5, 2);
                            curs_set(false);
                            print      = false;
                            scriptPath = saveScriptAs;
                            saveScript();
                            refreshView();
                            break;
                        case 'X':
                            move(5, 2);
                            curs_set(false);
                            print = false;
                            refreshView();
                            break;
                    }
                }
            }
            else {
                scriptPath = saveScriptAs;
                saveScript();
                move(5, 2);
                curs_set(false);
                refreshView();
            }
        }

        // allows user to edit the gain for the entire script or specified frame
        void ScriptTuner::editGain() {
            erase();
            box(stdscr, 0, 0);
            attron(A_BOLD);
            mvprintw(0, (COLS - 14) / 2, " Script Tuner ");
            mvprintw(3, 2, "Edit Gain");
            attroff(A_BOLD);
            mvprintw(5, 2, "For Entire Script:");
            mvprintw(6, 2, "All: ---.- Upper: ---.- Lower: ---.-");
            mvprintw(7, 2, "For Frame: %d", frame + 1);
            mvprintw(8, 2, "All: ---.- Upper: ---.- Lower: ---.-");
            mvprintw(10, 2, "Use X to exit Edit Gain");
            move(6, 7);
            curs_set(false);
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
            bool editGainRun       = true;
            mvchgat(YPOSITION[i][j], XPOSITION[i][j], 5, A_STANDOUT, 0, nullptr);

            while (editGainRun) {

                switch (getch()) {
                    case 'X': editGainRun = false; break;
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
                            userInputToFrame();
                            mvprintw(YPOSITION[i][j], XPOSITION[i][j], "%d", frame + 1);
                        }
                        else {
                            mvprintw(YPOSITION[i][j], XPOSITION[i][j], "     ");
                            move(YPOSITION[i][j], XPOSITION[i][j]);
                            newGain = userInputToGain();
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

                                // if user has entered the same gain in upper and lower then automatically prints value
                                // in both and dashes upper and lower
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
                }               // switch

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
            refreshView();
        }  // editGain()


        // checks user input is a number and converts it a number that becomes the new frame number
        void ScriptTuner::userInputToFrame() {
            std::string tempframe = userInput();
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
                    if ((size_t) tempframe2 <= script.frames.size()) {

                        frame = tempframe2 - 1;
                    }
                    else {
                        beep();
                    }
                }
                catch (std::invalid_argument) {
                    beep();
                }
            }
        }
        // converts valid user input to gain
        float ScriptTuner::userInputToGain() {
            std::string tempGain = userInput();
            try {
                if (!tempGain.empty()) {
                    float tempGain2 = stof(tempGain);
                    if (tempGain2 >= 0 && tempGain2 <= 100) {
                        return tempGain2;
                    }
                    else {
                        beep();
                    }
                }
            }
            catch (std::invalid_argument) {
                beep();
            }

            return std::numeric_limits<float>::quiet_NaN();
        }
    }  // namespace tools
}  // namespace behaviour
}  // namespace module
