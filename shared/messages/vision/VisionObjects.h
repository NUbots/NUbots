
#ifdef MESSAGES_VISION_VISION_OBJECTS_H
#define MESSAGES_VISION_VISION_OBJECTS_H

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

		class BallObject : VisionObject {
        public:
			BallObject() : VisionObject() {}			
		};

		class Goal : VisionObject {
        public:
			Goal() : VisionObject() {}
			enum Type{
				LEFT,
				RIGHT,
				UNKOWN
			} type;
		};

		class Obstacle : VisionObject {
        public:
			Obstacle() : VisionObject() {}
			float arcWidth;
		};	




		//Line objects:

		class FieldLine : VisionObject {
        public:
			FieldLine() : VisionObject() {}
		};

		class CornerPoint : VisionObject {
        public:
			CornerPoint() : VisionObject() {}
			enum Type{
				L_CORNER,
				T_CORNER,
				X_CORNER
			} type;
		};

		class CentreCircle : VisionObject {
        public:
			CentreCircle() : VisionObject() {}		
		};

		class LineObjects{
        public:
			LineObjects(){}
			std::vector<CentreCircle> centre_circles;
			std::vector<CornerPoint> corner_points;
			std::vector<FieldLine> field_lines;
		};

	}
}

#endif
