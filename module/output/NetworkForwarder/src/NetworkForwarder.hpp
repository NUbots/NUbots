#ifndef MODULE_OUTPUT_NETWORKFORWARDER_HPP
#define MODULE_OUTPUT_NETWORKFORWARDER_HPP

#include <map>
#include <memory>
#include <nuclear>

#include "utility/type_traits/has_id.hpp"

namespace module::output {

namespace id {
    /// @brief Returns the id field of data or, if id does not exist, 0
    template <typename T>
    std::enable_if_t<!utility::type_traits::has_id<T>::value, uint32_t> get(const T& /*data*/) {
        return 0;
    }

    template <typename T>
    std::enable_if_t<utility::type_traits::has_id<T>::value, uint32_t> get(const T& data) {
        return data.id;
    }
}  // namespace id

class NetworkForwarder : public NUClear::Reactor {

public:
    /// @brief Called by the powerplant to build and setup the NetworkForwarder reactor.
    explicit NetworkForwarder(std::unique_ptr<NUClear::Environment> environment);

private:
    struct {
        std::string target;
    } config;

    struct Handle {
        double period = std::numeric_limits<double>::max();
        std::map<int, NUClear::clock::time_point> last_message;
        ReactionHandle reaction;
    };

    template <typename T>
    void add_handle(const std::string& type) {

        auto handle = std::make_shared<Handle>();

        handle->reaction =
            on<Trigger<T>, Buffer<4>, Priority::LOW>()
                .then(type,
                      [this, handle](std::shared_ptr<const T> msg) {
                          using namespace std::chrono;  // NOLINT(google-build-using-namespace) fine in function scope
                          int id = id::get(*msg);

                          auto now = NUClear::clock::now();
                          if (handle->last_message.count(id) == 0
                              || duration_cast<duration<double>>(now - handle->last_message[id]).count()
                                     > handle->period) {
                              powerplant.emit_shared<Scope::NETWORK>(std::move(msg), config.target, false);
                              handle->last_message[id] = now;
                          }
                      })
                .disable();

        handles[type] = handle;
    }

    void register_handles();

    std::map<std::string, std::shared_ptr<Handle>> handles;
};

}  // namespace module::output

#endif  // MODULE_OUTPUT_NETWORKFORWARDER_HPP
