/*
 * This file is part of ScriptTuner.
 *
 * ScriptTuner is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ScriptTuner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ScriptTuner.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "ScriptTuner.h"
#include "messages/support/Configuration.h"
#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/motion/ServoWaypoint.h"
#include "utility/math/angle.h"
#include "utility/file/fileutil.h"
#include "utility/configuration/json/parse.h"
#include "utility/configuration/json/serialize.h"

#include <ncurses.h>
#include <sstream>

namespace modules {
    namespace behaviours {
        namespace tools {

            struct LockServo {};

            ScriptTuner::ScriptTuner(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)),
                    scriptPath("ERROR"),
                    frame(0),
                    selection(0),
                    angleOrGain(true),
                    running(true) {

                // Add a blank frame to start with
                script.frames.emplace_back();

                on<Trigger<CommandLineArguments>>([this](const std::vector<std::string>& args) {
                    if(args.size() == 2) {
                        scriptPath = args[1];

                        // Check if the script exists and load it if it does.
                        if(utility::file::exists(scriptPath)) {
                            log<NUClear::DEBUG>("Loading script: ", scriptPath, '\n');
                            loadScript(scriptPath);
                        }
                    }
                    else {
                        log<NUClear::DEBUG>("Error: Expected 2 arguments on argv found ", args.size(), '\n');
                        powerPlant->shutdown();
                    }
                });

                on<Trigger<LockServo>, With<messages::platform::darwin::DarwinSensors>>([this](const LockServo&, const messages::platform::darwin::DarwinSensors& sensors) {

                    auto id = selection < 2 ? 18 + selection : selection - 2;

                    messages::motion::Script::Frame::Target target;

                    target.id = static_cast<messages::platform::darwin::DarwinSensors::Servo::ID>(id);
                    target.position = sensors.servo[id].presentPosition;
                    target.gain = 100;

                    script.frames[frame].targets.push_back(target);

                    // Emit a waypoint so that the motor will go rigid at this angle
                    auto waypoint = std::make_unique<messages::motion::ServoWaypoint>();
                    waypoint->time = NUClear::clock::now();
                    waypoint->id = target.id;
                    waypoint->gain = target.gain;
                    waypoint->position = target.position;
                    emit(std::move(waypoint));
                });

                powerPlant->addServiceTask(NUClear::threading::ThreadWorker::ServiceTask(std::bind(std::mem_fn(&ScriptTuner::run), this),
                                                                                         std::bind(std::mem_fn(&ScriptTuner::kill), this)));
            }

            void ScriptTuner::run() {

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

                // Build our initial GUI
                refreshView();

                // Now we just loop forever
                while (running) {
                    // Get the character the user has typed
                    switch(getch()) {
                        case KEY_UP:        // Change selection up
                            selection = selection == 0 ? 19 : selection - 1;
                            break;
                        case KEY_DOWN:      // Change selection down
                            selection = (selection + 1) % 20;
                            break;
                        case 9:             // Swap between angle and gain
                        case KEY_LEFT:      // Swap between angle and gain
                        case KEY_RIGHT:     // Swap between angle and gain
                            angleOrGain = !angleOrGain;
                            break;
                        case ',':           // Move left a frame
                            activateFrame(frame == 0 ? frame : frame - 1);
                            break;
                        case '.':           // Move right a frame
                            activateFrame(frame == script.frames.size() - 1 ? frame : frame + 1);
                            break;
                        case '\n':          // Edit selected field
                        case KEY_ENTER:     // Edit selected field
                            editSelection();
                            break;
                        case ' ':           // Toggle lock mode
                            toggleLockMotor();
                            break;
                        case 'S':           // Save the current script
                            saveScript();
                            break;
                        case 'T':           // Edit this frames duration
                            editDuration();
                            break;
                        case 'N':           // New frame
                            newFrame();
                            break;
                        case 'D':           // Delete frame
                            deleteFrame();
                            break;
                    }

                    // Update whatever visual changes we made
                    refreshView();
                }
            }

            void ScriptTuner::activateFrame(int frame) {
                this->frame = frame;

                auto waypoints = std::make_unique<std::vector<messages::motion::ServoWaypoint>>();
                for(auto& target : script.frames[frame].targets) {
                    waypoints->push_back(messages::motion::ServoWaypoint {
                        NUClear::clock::now() + std::chrono::milliseconds(500)
                        , target.id
                        , target.position
                        , target.gain
                    });
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
                mvprintw(2, 2, "Script: %s", scriptPath.c_str());   // Output our scripts name
                mvprintw(3, 2, "Frames:");  // The frames section is filled out after this
                mvprintw(4, 2, "Duration: %d", // Output the selected frames duration
                         std::chrono::duration_cast<std::chrono::milliseconds>(script.frames[frame].duration).count());

                // Output all of our frame numbers and highlight the selected frame
                move(3, 10);
                for(size_t i = 0; i < script.frames.size(); ++i) {
                    if(i == frame) {
                        // Turn on highlighting to show this frame is selected
                        attron(A_STANDOUT);
                    }
                    printw(std::to_string(i + 1).c_str());
                    if(i == frame) {
                        // Turn off highlighting
                        attroff(A_STANDOUT);
                    }
                    printw(" ");
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
                    mvprintw(i + 6, 2, "U");

                    // Output the motor name
                    attron(A_BOLD);
                    mvprintw(i + 6, 4, MOTOR_NAMES[i]);
                    attroff(A_BOLD);

                    // Everything defaults to 0 angle and gain (unless we find one)
                    mvprintw(i + 6, 26, "Angle:  -.---  Gain: ---.-");
                }

                for(auto& target : script.frames[frame].targets) {
                    // Output that this frame is locked (we shuffle the head to the top of the list)
                    mvprintw(((static_cast<int>(target.id) + 2) % 20) + 6, 2, "L");

                    // Output this frames gain and angle
                    mvprintw(((static_cast<int>(target.id) + 2) % 20) + 6, 26, "Angle: %+.3f  Gain: %5.1f", target.position, target.gain);
                }

                // Highlight our selected point
                mvchgat(selection + 6, angleOrGain ? 26 : 41, angleOrGain ? 13 : 11, A_STANDOUT, 0, nullptr);

                // We finished building
                refresh();
            }

            void ScriptTuner::toggleLockMotor() {

                // This finds if we have this particular motor stored in the frame
                auto targetFinder = [=](const messages::motion::Script::Frame::Target& target) {
                                            return (static_cast<size_t>(target.id) + 2) % 20 == selection;
                };

                // See if we have this target in our frame
                auto it = std::find_if(std::begin(script.frames[frame].targets),
                                       std::end(script.frames[frame].targets),
                                       targetFinder);

                // If we don't then save our current motor position as the position
                if(it == std::end(script.frames[frame].targets)) {

                    emit<Scope::DIRECT>(std::make_unique<LockServo>());
                }
                else {
                    // Remove this frame
                    script.frames[frame].targets.erase(it);

                    // Emit a waypoint so that the motor will turn off gain (go limp)
                    auto waypoint = std::make_unique<messages::motion::ServoWaypoint>();
                    waypoint->time = NUClear::clock::now();
                    waypoint->id = static_cast<messages::platform::darwin::DarwinSensors::Servo::ID>(selection < 2 ? 18 + selection : selection - 2);
                    waypoint->gain = 0;
                    waypoint->position = 0;
                    emit(std::move(waypoint));
                }
            }

            void ScriptTuner::newFrame() {
                // Make a new frame before our current with our current set of motor angles and unlocked/locked status
                auto newFrame = script.frames[frame];
                script.frames.insert(script.frames.begin() + frame, newFrame);
            }

            void ScriptTuner::deleteFrame() {
                // Delete our current frame and go to the one before this one, if this is the last frame then ignore
                if(script.frames.size() > 1) {
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
                while(true) {
                    auto ch = getch();
                    switch(ch) {
                        case 27:
                            return "";
                        case '\n':
                        case KEY_ENTER:
                            return chars.str();
                            break;
                        default:
                            chars << static_cast<char>(ch);
                            addch(ch);
                            break;
                    }
                }
            }

            void ScriptTuner::loadScript(const std::string& path) {
                script = utility::configuration::json::parse(utility::file::loadFromFile(path));
            }

            void ScriptTuner::saveScript() {
                utility::file::writeToFile(scriptPath, utility::configuration::json::serialize(script));
            }

            void ScriptTuner::editDuration() {

                // Move to the correct position and erase the old duration
                move(4, 12);
                for(int i = 0; i < 10; ++i) {
                    addch(' ');
                }
                move(4, 12);

                // Get the users input
                std::string result = userInput();

                // If we have a result
                if(!result.empty()) {
                    try {
                        int num = stoi(result);
                        script.frames[frame].duration = std::chrono::milliseconds(num);
                    }
                    // If it's not a number then ignore and beep
                    catch(std::invalid_argument) {
                        beep();
                    }
                }
            }

            void ScriptTuner::editSelection() {

                // Erase our old text
                mvprintw(selection + 6, angleOrGain ? 33 : 46, "      ");

                // Move to our point
                move(selection + 6, angleOrGain ? 33 : 46);

                // Get the users input
                std::string result = userInput();

                // If we have a result
                if(!result.empty()) {
                    try {
                        double num = stod(result);

                        // This finds if we have this particular motor stored in the frame
                        auto targetFinder = [=](const messages::motion::Script::Frame::Target& target) {
                                                    return (static_cast<size_t>(target.id) + 2) % 20 == selection;
                        };

                        // See if we have this target in our frame
                        auto it = std::find_if(std::begin(script.frames[frame].targets),
                                               std::end(script.frames[frame].targets),
                                               targetFinder);

                        // If we don't have this frame
                        if(it == std::end(script.frames[frame].targets)) {
                            it = script.frames[frame].targets.emplace(std::end(script.frames[frame].targets));
                            auto id = selection < 2 ? 18 + selection : selection - 2;
                            it->id = static_cast<messages::platform::darwin::DarwinSensors::Servo::ID>(id);
                            it->position = 0;
                            it->gain = 0;
                        }

                        // If we are entering an angle
                        if (angleOrGain) {

                            // Normalize our angle to be between -pi and pi
                            num = utility::math::angle::normalizeAngle(num);
                            /*num = fmod(num + M_PI, M_PI * 2);
                            if (num < 0)
                                num += M_PI * 2;
                            num -= M_PI;*/

                            it->position = num;
                            // Convert our angle to be between -pi and pi
                        }
                        // If it is a gain
                        else {
                            if(num >= 0 && num <= 100) {
                                it->gain = num;
                            }
                            else {
                                beep();
                            }
                            // Check if the value is < 0 or > 100
                        }
                    }
                    // If it's not a number then ignore and beep
                    catch(std::invalid_argument) {
                        beep();
                    }
                }
            }

            void ScriptTuner::kill() {
                running = false;
                endwin();
            }
            
        }  // tools
    }  // behaviours
}  // modules