/*
 * This file is part of FeatureDetector.
 *
 * FeatureDetector is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * FeatureDetector is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FeatureDetector.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGES_VISION_VISIONOBJECTS_H
#define MESSAGES_VISION_VISIONOBJECTS_H

#include <armadillo> 
#include <nuclear>

namespace messages {
	namespace vision {

		class VisionObject {
        public:
            VisionObject() {}

			arma::vec3 sphericalFromNeck;	//neckRelativeRadial
			arma::vec3 sphericalError;
			arma::vec2 screenAngular;	//Polar around view vector on image
			arma::vec2 screenCartesian;
			arma::vec2 sizeOnScreen;
			NUClear::clock::time_point timestamp;
		};

		class Ball : public VisionObject {
        public:
			Ball() : VisionObject() {}			
		};

		class Goal : public VisionObject {
        public:
			Goal() : VisionObject() {}
			enum Type{
				LEFT,
				RIGHT,
				UNKOWN
			} type;
		};

		class Obstacle : public VisionObject {
        public:
			Obstacle() : VisionObject() {}
			float arcWidth;
		};	




		//Line objects:

		class FieldLine : public VisionObject {
        public:
			FieldLine() : VisionObject() {}
		};

		class CornerPoint : public VisionObject {
        public:
			CornerPoint() : VisionObject() {}

			enum Type {
				L_CORNER,
				T_CORNER,
				X_CORNER,
                INVALID
			} type;
		};

		class CentreCircle : public VisionObject {
        public:
			CentreCircle() : VisionObject() {}		
		};

		class LineObjects{
        public:
			LineObjects() {}
			std::vector<CentreCircle> centre_circles;
			std::vector<CornerPoint> corner_points;
			std::vector<FieldLine> field_lines;
		};

	}
}

#endif // MESSAGES_VISION_VISIONOBJECTS_H
