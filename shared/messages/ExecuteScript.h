#ifndef MESSAGES_EXECUTESCRIPT_H
#define MESSAGES_EXECUTESCRIPT_H

#include <NUClear.h>

namespace messages {
    struct ExecuteScript {
        ExecuteScript(const std::string& scriptName, NUClear::clock::time_point start = NUClear::clock::now()) : script(scriptName), start(start) {};
        std::string script;
        NUClear::clock::time_point start;
    };
}

#endif    /* MODULES_EXECUTESCRIPT_H */

