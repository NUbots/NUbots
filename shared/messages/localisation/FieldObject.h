#ifndef MESSAGES_LOCALISATIONFIELDOBJECT
#define	MESSAGES_LOCALISATIONFIELDOBJECT

namespace messages {
	namespace localisation {
		struct FieldObject {
			std::string name;
			float wm_x;
			float wm_y;
			float sd_x;
			float sd_y;
			float sr_xx;
			float sr_xy;
			float sr_yy;
			bool lost;
		};
	}
}

#endif
