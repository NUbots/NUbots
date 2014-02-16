/*
 * This file is part of Localisation.
 *
 * Localisation is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Localisation is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Localisation.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_MULTIMODALROBOTMODEL_H
#define MODULES_MULTIMODALROBOTMODEL_H

#include <nuclear>

namespace modules {
namespace localisation {

    class RobotModel {
    private:
        bool active_;

    public:
        bool active() { return active_; }
        void set_active(bool active) { active_ = active; }
    };

    class MultiModalRobotModel {
    public:
        MultiModalRobotModel();

        unsigned int RemoveInactiveModels();
        unsigned int RemoveInactiveModels(std::vector<RobotModel>& container);
        void PruneModels();
        void RemoveSimilarModels();
        void NormaliseAlphas();

        void LandmarkUpdate();
        void MultipleLandmarkUpdate();
        // int AmbiguousLandmarkUpdate(
        //     AmbiguousObject &ambiguous_object,
        //     const std::vector<StationaryObject*>& possible_objects);

    private:
        int PruneViterbi(unsigned int order);
        // int AmbiguousLandmarkUpdateExhaustive(
        //     AmbiguousObject &ambiguous_object,
        //     const std::vector<StationaryObject*>& possible_objects);

        std::vector<RobotModel> robot_models_;
    };
}
}
#endif
