#ifndef MESSAGES_SOUNDCHUNK_H
#define	MESSAGES_SOUNDCHUNK_H

#include <NUClear.h>
#include <vector>
#include <cstdint>

namespace messages {
    struct SoundChunk {
        
        NUClear::clock::time_point endTime;
        std::vector<int16_t> data;
    };
}

#endif	/* MODULES_SOUNDCHUNK_H */

