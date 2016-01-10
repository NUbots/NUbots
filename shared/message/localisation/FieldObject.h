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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGE_LOCALISATIONFIELDOBJECT
#define MESSAGE_LOCALISATIONFIELDOBJECT

#include <armadillo>
#include <nuclear>

namespace message {
    namespace localisation {

        // Sent to NUbugger
        struct FieldObject {
            std::string name;

            struct Model {
                double wm_x; //world model x
                double wm_y; //world model y
                double heading; //direction
                double sd_heading;  //stdev
                double sd_x;
                double sd_y;
                double sr_xx;   //covariance
                double sr_xy;
                double sr_yy;
                bool lost;
            };

            std::vector<Model> models;

        };

        class LocalisationObject {
        public:
            LocalisationObject() {}

            arma::vec2 position;
            arma::mat22 position_cov;

            NUClear::clock::time_point last_measurement_time;
        };

        class Ball : public LocalisationObject {
        public:
            Ball() : LocalisationObject() {}
            arma::vec2 velocity;
            bool world_space; // Ball will always be in robot space except
                              // when sent from MockRobot.
        };

        class Self : public LocalisationObject {
        public:
            Self() : LocalisationObject() {}

            arma::vec2 heading;     //robot face direction (vector 2)
            arma::vec2 velocity;    //robot velocity (vector 2)
            arma::mat22 robot_to_world_rotation;    //??might not be useful
        };
    }
}

#endif
