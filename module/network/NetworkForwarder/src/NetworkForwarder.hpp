#ifndef MODULE_NETWORK_NETWORKFORWARDER_HPP
#define MODULE_NETWORK_NETWORKFORWARDER_HPP

#include <map>
#include <memory>
#include <nuclear>

#include "utility/type_traits/has_id.hpp"

namespace module::network {

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
        struct Handle {
            struct TargetHandle {
                double period = std::numeric_limits<double>::max();
                std::map<int, NUClear::clock::time_point> last_message;
            };

            std::map<std::string, TargetHandle> targets;
            ReactionHandle reaction;
        };

        template <typename T>
        void add_handle(const std::string& type) {

            auto handle = std::make_shared<Handle>();

            // We put a buffer here so that under normal operation this reaction isn't impeded.
            // However if because of the low priority other modules are taking all the thread time this module won't
            // queue indefinitely killing all the ram in the system.
            handle->reaction =
                on<Trigger<T>, Buffer<10>, Priority::LOW>()
                    .then(
                        type,
                        [this, type, handle](std::shared_ptr<const T> msg) {
                            using namespace std::chrono;  // NOLINT(google-build-using-namespace) fine in function scope
                            int id = id::get(*msg);

                            auto now = NUClear::clock::now();

                            // Loop through all the targets that are interested in this type
                            for (auto& target_handle : handle->targets) {
                                const auto& target = target_handle.first;
                                auto& h            = target_handle.second;

                                if (!h.last_message.contains(id)
                                    || duration_cast<duration<double>>(now - h.last_message[id]).count() > h.period) {
                                    powerplant.emit_shared<Scope::NETWORK>(std::move(msg), target, false);
                                    h.last_message[id] = now;
                                    log<NUClear::TRACE>("Forwarding", type, "to", target);
                                }
                            }
                        })
                    .disable();

            handles[type] = handle;
        }

        void register_handles();

        std::map<std::string, std::shared_ptr<Handle>> handles;
    };

}  // namespace module::network

#endif  // MODULE_NETWORK_NETWORKFORWARDER_HPP
