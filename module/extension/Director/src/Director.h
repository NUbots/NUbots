#ifndef MODULE_EXTENSION_DIRECTOR_H
#define MODULE_EXTENSION_DIRECTOR_H

#include <nuclear>
#include <typeindex>

namespace module {
namespace extension {

    class Director : public NUClear::Reactor {

    private:
        struct Provider {
            struct WhenCondition {
                WhenCondition(const std::type_index& type, bool (*expr)(const int&), int (*current)())
                    : type(type), expr(expr), current(current) {}
                std::type_index type;      // The type of state that this condition is checking
                bool (*expr)(const int&);  // Expression to determine if the state is valid
                int (*current)();          // Function to get the current state
            };

            enum Type { NORMAL, ENTERING, LEAVING } type;
            std::shared_ptr<NUClear::threading::Reaction> reaction;
            std::vector<WhenCondition> when;
            std::map<std::type_index, int> causing;

            Provider(const Type& type, const std::shared_ptr<NUClear::threading::Reaction>& reaction)
                : type(type), reaction(reaction) {}
        };

        void add_provider(const std::type_index& data_type,
                          const std::shared_ptr<NUClear::threading::Reaction>& r,
                          const Provider::Type& provider_type);
        void remove_provider(const uint64_t& id);

    public:
        /// @brief Called by the powerplant to build and setup the Director reactor.
        explicit Director(std::unique_ptr<NUClear::Environment> environment);

    private:
        std::multimap<std::type_index, Provider> providers;
        std::map<uint64_t, std::multimap<std::type_index, Provider>::iterator> reactions;
    };

}  // namespace extension
}  // namespace module

#endif  // MODULE_EXTENSION_DIRECTOR_H
