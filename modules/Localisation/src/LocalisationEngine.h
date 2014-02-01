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

#ifndef MODULES_LOCALISATIONENGINE_H
#define MODULES_LOCALISATIONENGINE_H

#include <nuclear>

#include "MultiModalRobotModel.h"

namespace modules {
    class LocalisationEngine {
    	public:
    	LocalisationEngine();

        void RecordMeasurement(Measurement m, time_t timestamp);

        void SwapMeasurementBuffers();

        void TimeUpdate();

        void ObjectUpdate();

    private:
		// TODO: Consider extracting the robot models into an actual class,
		// that handles the robot model without regard to whether or not
		// it is represented by a multi-modal distribution. (e.g. methods like
		// removeInactiveModels(), would be instance methods  of this new
		// class)
    	MultiModalRobotModel robot_models_;
    };
}
#endif
