#ifndef MODULE_EXTENSION_DIRECTOR_H
#define MODULE_EXTENSION_DIRECTOR_H

#include <nuclear>
#include <typeid>

namespace module {
namespace extension {

    class Director : public NUClear::Reactor {

    private:
        struct Provider {

            enum Type { NORMAL, ENTERING, LEAVING } type;
            std::shared_ptr<NUClear::threading::Reaction> reaction;

            Provider(const Type& type, const std::shared_ptr<NUClear::threading::Reaction>& reaction)
                : type(type), reaction(reaction) {}

            // TODO Causings
            // TODO When conditions
        };

        void add_provider(const std::type_index& data_type,
                          const std::shared_ptr<NUClear::threading::Reaction>& r,
                          const Provider::Type& provider_type);

    public:
        /// @brief Called by the powerplant to build and setup the Director reactor.
        explicit Director(std::unique_ptr<NUClear::Environment> environment);

    private:
        std::multimap<std::type_index, Provider> providers;
        std::map<uint64_t, std::multimap<std::type_index, std::vector<Provider>>::iterator> reactions;
    };

}  // namespace extension
}  // namespace module

#endif  // MODULE_EXTENSION_DIRECTOR_H
