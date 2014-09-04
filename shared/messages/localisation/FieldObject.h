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

#ifndef MESSAGES_LOCALISATIONFIELDOBJECT
#define MESSAGES_LOCALISATIONFIELDOBJECT

#include <armadillo>

namespace messages {
    namespace localisation {

        // Sent to NUbugger
        struct FieldObject {
            std::string name;

            struct Model {
                double wm_x;
                double wm_y;
                double heading;
                double sd_heading;
                double sd_x;
                double sd_y;
                double sr_xx;
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
            arma::vec2 heading;
            arma::vec2 velocity;
            arma::mat22 robot_to_world_rotation;
        };

        template<class T> class Mock {
        public:
            Mock() : data() {}
            Mock(T t) : data(t) {}
            T data;
        };
    }
}

#endif
