
#ifndef UTILITY_MOTION_KINEMATICS_ROBOTMODELS_H
#define UTILITY_MOTION_KINEMATICS_ROBOTMODELS_H

namespace utility{
	namespace motion{
		namespace kinematics{

			enum class Side : bool {
			    LEFT = true,
			    RIGHT = false
		    };

			class DarwinModel{
			public:
                
				class Leg {
				public:
					static constexpr float LENGTH_BETWEEN_LEGS = 0.074;
			        static constexpr float HIP_OFFSET_Z = 0.034;
			        static constexpr float HIP_OFFSET_X = 0.008;
					static constexpr float UPPER_LEG_LENGTH = 0.093;
					static constexpr float LOWER_LEG_LENGTH = 0.093;
					static constexpr float FOOT_HEIGHT = 0.0335;
		


					static constexpr int LEFT_TO_RIGHT_HIP_YAW = 	   -1;
					static constexpr int LEFT_TO_RIGHT_HIP_ROLL = 	   -1;
					static constexpr int LEFT_TO_RIGHT_HIP_PITCH =      1;
					static constexpr int LEFT_TO_RIGHT_KNEE =           1;
					static constexpr int LEFT_TO_RIGHT_ANKLE_PITCH =    1;
					static constexpr int LEFT_TO_RIGHT_ANKLE_ROLL =    -1;
				};

				class Head {
				public:
					static constexpr float NECK_BASE_POS_FROM_ORIGIN[3] = {0.013, 0, 0.11};
					static constexpr float NECK_LENGTH = 0.0305;
					static constexpr float NECK_TO_CAMERA[3] = {0.033,0,0.033};
					static constexpr float CAMERA_DECLINATION_ANGLE_OFFSET = 0;
				};

				class MassModel {
					static constexpr arma::vec4[] masses = {
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0}),
						arma::vec4({0,0,0,0})
					}
				};

				static constexpr float TEAMDARWINCHEST_TO_ORIGIN = 0.096 - Leg::HIP_OFFSET_Z; //Taken from team darwin OPkinematics.cpp : hipOffsetZ = .096; 
			};
		}
	}
}

#endif