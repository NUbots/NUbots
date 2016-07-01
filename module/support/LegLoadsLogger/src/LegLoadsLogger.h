#ifndef MODULE_SUPPORT_LEGLOADSLOGGER_H
#define MODULE_SUPPORT_LEGLOADSLOGGER_H

#include <nuclear>

namespace module {
namespace support {

    class LegLoadsLogger : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the LegLoadsLogger reactor.
        explicit LegLoadsLogger(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_SUPPORT_LEGLOADSLOGGER_H
