#ifndef MESSAGES_NUBUGGERDATAPOINT_H
#define	MESSAGES_NUBUGGERDATAPOINT_H

namespace messages {

	namespace NUbugger {

		struct DataPoint {
			std::string label;
			std::vector<float> values;
		};

	}
	
}

#endif

