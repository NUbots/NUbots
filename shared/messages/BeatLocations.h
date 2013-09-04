#ifndef MESSAGES_BEATLOCATIONS_H
#define	MESSAGES_BEATLOCATIONS_H

#include <NUClear.h>

namespace messages {
    struct BeatLocations {
        NUClear::clock::time_point firstBeatTime;
        std::chrono::milliseconds beatPeriod;
        int numBeats;
    };
}

#endif	/* MODULES_SOUNDCHUNK_H */

