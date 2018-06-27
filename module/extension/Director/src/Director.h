#ifndef MODULE_EXTENSION_DIRECTOR_H
#define MODULE_EXTENSION_DIRECTOR_H

#include <nuclear>
#include <typeid>

namespace module {
namespace extension {

    class Director : public NUClear::Reactor {

    private:
        struct Provider {
            enum Type { UNKNOWN, NORMAL, LEAVING, ENTERING } type;

            std::shared_ptr<NUClear::threading::Reaction> reaction;

            // TODO Causings
            // TODO When conditions
        };

    public:
        /// @brief Called by the powerplant to build and setup the Director reactor.
        explicit Director(std::unique_ptr<NUClear::Environment> environment);

    private:
        std::map<std::type_index, std::vector<Provider>> providers;
    };

}  // namespace extension
}  // namespace module

#endif  // MODULE_EXTENSION_DIRECTOR_H
