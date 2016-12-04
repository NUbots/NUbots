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

#ifndef MODULES_VISION_SLAME_H
#define MODULES_VISION_SLAME_H

#include <nuclear>
#include "utility/vision/MockFeatureExtractor.h"
#include "message/input/Image.h"
#include "message/localisation/FieldObject.h"
#include "message/input/Sensors.h"
#include "message/vision/VisionObjects.h"
#include "SLAMEModule.h"

namespace module {
    namespace vision {
        // template class SLAMEModule<utility::vision::ORBFeatureExtractor>;
        // template class SLAMEModule<utility::vision::MockFeatureExtractor>;

        enum FeatureExtractorType{
            ORB,
            LSH,
            MOCK
        };
        /**
         * TODO document
         *
         * @author Jake Fountain
         */
        class SLAME : public NUClear::Reactor {
        private:
            FeatureExtractorType FEATURE_EXTRACTOR_TYPE;
            int MAX_FEATURE_MATCHES;

            //SLAMEModule<utility::vision::ORBFeatureExtractor> ORBModule;
            SLAMEModule<utility::vision::MockFeatureExtractor> MockSLAMEModule;

            ReactionHandle fakeLocalisationHandle;
            double FAKE_LOCALISATION_PERIOD = 1;
            double FAKE_LOCALISATION_RADIUS = 1;
            NUClear::clock::time_point start_time;
        public:
            explicit SLAME(std::unique_ptr<NUClear::Environment> environment);
        };
    }
}
#endif

