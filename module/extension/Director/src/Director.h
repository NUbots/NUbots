#ifndef MODULE_EXTENSION_DIRECTOR_H
#define MODULE_EXTENSION_DIRECTOR_H

#include <nuclear>

namespace module {
namespace extension {

    class Director : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Director reactor.
        explicit Director(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_EXTENSION_DIRECTOR_H
