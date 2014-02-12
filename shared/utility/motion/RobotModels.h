

namespace utility{
	namespace motion{
		namespace kinematics{

			enum class Side : bool {
			    LEFT = true;
			    RIGHT = false;
		    };

			class DarwinModel{
			public:
				class Leg {
				public:
					static constexpr float LENGTH_BETWEEN_LEGS = 0.074;
			        static constexpr float DISTANCE_FROM_BODY_TO_HIP_JOINT = 0.034;
					static constexpr float UPPER_LEG_LENGTH = 0.093;
					static constexpr float LOWER_LEG_LENGTH = 0.093;


					static constexpr int LEFT_TO_RIGHT_HIP_YAW = 	    1;
					static constexpr int LEFT_TO_RIGHT_HIP_ROLL = 	   -1;
					static constexpr int LEFT_TO_RIGHT_HIP_PITCH =      1;
					static constexpr int LEFT_TO_RIGHT_KNEE =           1;
					static constexpr int LEFT_TO_RIGHT_ANKLE_PITCH =    1;
					static constexpr int LEFT_TO_RIGHT_ANKLE_ROLL =    -1;
				};

				class Head {
				public:

				};
			};
		}
	}
}