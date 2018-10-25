#ifndef MODULE_BEHAVIOUR_TOOLS_PYTHONSCRIPT_H
#define MODULE_BEHAVIOUR_TOOLS_PYTHONSCRIPT_H

#include <nuclear>

namespace module {
namespace behaviour {
    namespace tools {

        class PythonScript : public NUClear::Reactor {

        public:
            /// @brief Called by the powerplant to build and setup the PythonScript reactor.
            explicit PythonScript(std::unique_ptr<NUClear::Environment> environment);

        private:
            /// Our ID for subsumption
            const size_t id;
        };

    }  // namespace tools
}  // namespace behaviour
}  // namespace module

#endif  // MODULE_BEHAVIOUR_TOOLS_PYTHONSCRIPT_H
