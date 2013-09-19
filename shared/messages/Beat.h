#ifndef MESSAGES_BEAT_H
#define	MESSAGES_BEAT_H

#include <NUClear.h>

namespace messages {
    struct Beat {
        NUClear::clock::time_point time;
        NUClear::clock::duration period;
    };
}

#endif  // MESSAGES_BEAT_H

