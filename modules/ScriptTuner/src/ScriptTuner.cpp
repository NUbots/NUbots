/*
 * This file is part of NCursesReactor.
 *
 * NCursesReactor is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * NCursesReactor is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with NCursesReactor.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Trent Houliston <trent@houliston.me>
 */

#include "ScriptTuner.h"
#include "messages/DarwinSensors.h"
#include "messages/ServoWaypoint.h"

#include <ncurses.h>

namespace modules {

    ScriptTuner::ScriptTuner(NUClear::PowerPlant* plant) : Reactor(plant) {

        on < Trigger < messages::DarwinSensors >> ([this](const messages::DarwinSensors & sensors) {
            // Update our curses display
        });

        powerPlant->addServiceTask(
                                   NUClear::Internal::ThreadWorker::ServiceTask(std::bind(std::mem_fn(&ScriptTuner::run), this),
                                                                                std::bind(std::mem_fn(&ScriptTuner::kill), this)));


        /*
         *                      SCRIPT TUNER
         * Script   Script name
         * FRAME 1 2 3 4 5 6 7 8 9 10
         * [Run] [Step]
         * Duration: 1000
         *
         * U Head Pan               0.00 radians
         * U Head Tilt              0.00 radians
         * L Right Shoulder Pitch   0.00 radians
         * U Left Shoulder Pitch    0.00 radians
         * L Right Shoulder Roll    0.00 radians
         * L Left Shoulder Roll     0.00 radians
         * L Right Elbow            0.00 radians
         * L Left Elbow             0.00 radians
         * L Right Hip Yaw          0.00 radians
         * L Left Hip Yaw           0.00 radians
         * L Right Hip Roll         0.00 radians
         * L Left Hip Roll          0.00 radians
         * L Right Hip Pitch        0.00 radians
         * L Left Hip Pitch         0.00 radians
         * L Right Knee             0.00 radians
         * L Left Knee              0.00 radians
         * L Right Ankle Pitch      0.00 radians
         * L Left Ankle Pitch       0.00 radians
         * L Right Ankle Roll       0.00 radians
         * L Left Ankle Roll        0.00 radians
         */

    }

    enum class Selection {

        SCRIPTNAME,
        FRAME,
        DURATION,
        HEAD_PAN,
        HEAD_TILT,
        R_SHOULDER_PITCH,
        L_SHOULDER_PITCH,
        R_SHOULDER_ROLL,
        L_SHOULDER_ROLL,
        R_ELBOW,
        L_ELBOW,
        R_HIP_YAW,
        L_HIP_YAW,
        R_HIP_ROLL,
        L_HIP_ROLL,
        R_HIP_PITCH,
        L_HIP_PITCH,
        R_KNEE,
        L_KNEE,
        R_ANKLE_PITCH,
        L_ANKLE_PITCH,
        R_ANKLE_ROLL,
        L_ANKLE_ROLL
    };

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

        // Build our window

        // Outer box
        box(stdscr, 0, 0);

        // Write our title
        attron(A_BOLD);
        mvprintw(0, (COLS - 14) / 2, " Script Tuner ");
        attroff(A_BOLD);

        // Top sections
        mvprintw(2, 2, "Script: NEW");
        mvprintw(3, 2, "Frames: 1");
        mvprintw(4, 2, "Duration: 1000");

        // Each motor
        const char* motors[] = {
                                "Head Pan",
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
                                "Left Ankle Roll"
        };
        for (int i = 0; i < 20; ++i) {

            mvprintw(i + 6, 2, "L");
            attron(A_BOLD);
            mvprintw(i + 6, 4, motors[i]);
            attroff(A_BOLD);


            mvprintw(i + 6, 26, "Angle: 0.000  Gain: 0.0");
        }

        // We finished building
        refresh();


        // Setup our state variables
        Selection selection = Selection::SCRIPTNAME;
        bool editing;

        // Now we just loop forever
        while (running) {
            int ch = getch();

            if(editing) {
                switch(ch) {
                    case KEY_EXIT:
                        // stop editing without saving
                        break;
                    case KEY_ENTER:
                        // Save the value
                        break;
                }
            }
            else {
                // This is our normal mode
                switch(ch) {
                    // Up down selection
                    case KEY_UP:
                    case KEY_DOWN:
                    // Left right frame
                    case KEY_LEFT:
                    case KEY_RIGHT:
                    case 'n': // New frame after
                    case 'N': // New frame before
                    case 'd': // Delete frame
                        break;
                }
            }
        }
    }

    void ScriptTuner::kill() {
        running = false;
        endwin();
    }
}
