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

#ifndef MODULES_MMKFROBOTLOCALISATIONENGINE_H
#define MODULES_MMKFROBOTLOCALISATIONENGINE_H

#include <nuclear>

#include "messages/vision/VisionObjects.h"
#include "utility/localisation/FieldDescription.h"
#include "MultiModalRobotModel.h"
#include "utility/localisation/FieldDescription.h"
#include "utility/localisation/LocalisationFieldObject.h"

namespace modules {
namespace localisation {

    class MMKFRobotLocalisationEngine {
        public:

        MMKFRobotLocalisationEngine() { }

        void TimeUpdate(time_t current_time);

        std::vector<utility::localisation::LocalisationFieldObject> GetPossibleObjects(
            const messages::vision::Goal& ambiguous_object);
        
        void ProcessAmbiguousObjects(
            const std::vector<messages::vision::Goal>& ambiguous_objects);

        void IndividualStationaryObjectUpdate(
            const std::vector<messages::vision::Goal>& goals,
            float time_increment);

        void ProcessObjects(const std::vector<messages::vision::Goal>& goals);
        
        std::shared_ptr<utility::localisation::FieldDescription> field_description() {
            return field_description_;
        };
        void set_field_description(std::shared_ptr<utility::localisation::FieldDescription> desc) {
            field_description_ = desc;
        };

    // private:
        /// Contains the dimensions of the field
        std::shared_ptr<utility::localisation::FieldDescription> field_description_;

        MultiModalRobotModel robot_models_;
    };
}
}
#endif
