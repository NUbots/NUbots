#ifndef MESSAGES_EXECUTESCRIPT_H
#define MESSAGES_EXECUTESCRIPT_H

#include <NUClear.h>

namespace messages {
    struct ExecuteScript {
        std::string script;
        NUClear::clock::time_point start = NUClear::clock::now();
    };
}

#endif    /* MODULES_EXECUTESCRIPT_H */

