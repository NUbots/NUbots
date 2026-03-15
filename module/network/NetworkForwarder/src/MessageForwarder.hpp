#ifndef MODULE_NETWORK_NETWORK_FORWARDER_MESSAGE_FORWARDER_HPP
#define MODULE_NETWORK_NETWORK_FORWARDER_MESSAGE_FORWARDER_HPP

#include <chrono>
#include <fmt/format.h>
#include <memory>
#include <string>
#include <utility>

#include "Forwarder.hpp"
#include "NetworkForwarder.hpp"

#include "utility/type_traits/has_id.hpp"

namespace module::network {

    namespace id {
        /// Returns the id field of data or, if id does not exist, 0
        template <typename T>
        static uint32_t get(const T& data) {
            if constexpr (utility::type_traits::has_id<T>::value) {
                return data.id;
            }
            return 0;
        }
    }  // namespace id

    /**
     * A message forwarder for a specific message type
     */
    template <typename T>
    class MessageForwarder;

    template <>
    class MessageForwarder<void> : public Forwarder {};

    template <typename T>
    class MessageForwarder : public MessageForwarder<void> {
    public:
        MessageForwarder(NetworkForwarder& reactor, std::string target, std::string message_name)
            : reactor(reactor), target(std::move(target)), type(std::move(message_name)) {}

        // Delete copy and move operations since we have a destructor
        MessageForwarder(const MessageForwarder&)            = delete;
        MessageForwarder(MessageForwarder&&)                 = delete;
        MessageForwarder& operator=(const MessageForwarder&) = delete;
        MessageForwarder& operator=(MessageForwarder&&)      = delete;

        ~MessageForwarder() override {
            reaction_handle_in.unbind();
            reaction_handle_out.unbind();
        }

        [[nodiscard("getter")]] std::string message_name() const override {
            return type;
        }

        [[nodiscard("getter")]] std::string target_name() const override {
            return target;
        }

        void configure(const Config& new_config) override {
            // If there are no changes to the configuration, do nothing
            if (new_config == config) {
                return;
            }

            // Log the configuration change
            std::string direction_str = !new_config.send && !new_config.recv  ? " | "
                                        : !new_config.send && new_config.recv ? "<- "
                                        : new_config.send && !new_config.recv ? " ->"
                                                                              : "<->";
            reactor.log<NUClear::LogLevel::INFO>("Configuring", type, "( Local", direction_str, target, ")");

            // Handle inbound reaction changes (recv)
            if (new_config.recv != config.recv) {
                if (new_config.recv && !reaction_handle_in.enabled()) {
                    // Need to bind inbound reaction
                    bind_inbound_reaction();
                }
                else if (!new_config.recv && reaction_handle_in.enabled()) {
                    // Need to unbind inbound reaction
                    reaction_handle_in.unbind();
                }
            }

            // Handle outbound reaction changes (send)
            if (new_config.send != config.send) {
                if (new_config.send && !reaction_handle_out.enabled()) {
                    // Need to bind outbound reaction
                    bind_outbound_reaction();
                }
                else if (!new_config.send && reaction_handle_out.enabled()) {
                    // Need to unbind outbound reaction
                    reaction_handle_out.unbind();
                }
            }

            // Update stored configuration
            config = new_config;
            period = std::chrono::duration_cast<NUClear::clock::duration>(
                std::chrono::duration<double>(config.rate_limit == 0 ? 0.0 : 1.0 / config.rate_limit));
        }

        void send_out(const uint32_t& id) override {
            if (!config.send) {  // Only send if outbound forwarding is enabled
                return;
            }

            const std::lock_guard lock(mutex);
            if (id == 0) {
                // Send all subtypes
                for (auto& [subtype_id, cache] : caches) {
                    reactor.powerplant.emit_shared<NetworkScope>(cache.message, target, true);
                    NUClear::log<NUClear::LogLevel::TRACE>("Sending", type, "to", target);
                }
            }
            else if (caches.contains(id)) {
                // Send the specific subtype
                reactor.powerplant.emit_shared<NetworkScope>(caches.at(id).message, target, true);
                NUClear::log<NUClear::LogLevel::TRACE>("Sending", type, "subtype", id, "to", target);
            }
            else {
                throw std::runtime_error(fmt::format("The subtype {} has not been emitted for message {}", id, type));
            }
        }

    private:
        // Reaction types
        template <typename U>
        using Trigger = NUClear::dsl::word::Trigger<U>;
        template <typename U>
        using Network = NUClear::dsl::word::Network<U>;

        // Reaction options
        using Inline = NUClear::dsl::word::Inline;

        // Emit types
        template <typename U>
        using NetworkScope = NUClear::dsl::word::emit::Network<U>;
        template <typename U>
        using LocalScope = NUClear::dsl::word::emit::Local<U>;

        /// The time at which to forward the most recent message for a given message type
        struct CachedMessage {
            /// The message to forward
            std::shared_ptr<const T> message;
            /// The time the message was last forwarded
            NUClear::clock::time_point last_sent{};
        };

        /// Bind the reaction for forwarding local messages to the network
        void bind_outbound_reaction() {
            reaction_handle_out =
                reactor.on<Trigger<T>, Inline::ALWAYS>().then([this](const std::shared_ptr<const T>& msg) {
                    // Only forward this message out if it isn't a network message that was just forwarded locally
                    if (msg.get() == last_inbound_msg) {
                        return;
                    }

                    // Get the id from the message (or 0 if it doesn't have one)
                    uint32_t id = id::get(*msg);
                    auto now    = NUClear::clock::now();

                    // Whether to forward the message or not
                    bool forward = false;

                    /* Mutex Scope */ {
                        // Lock the mutex for message schedule access
                        const std::lock_guard lock(mutex);

                        // Get or create the scheduled message for this id
                        auto& cache =
                            caches.try_emplace(id, CachedMessage{msg, NUClear::clock::time_point::min()}).first->second;

                        // Update the scheduled message with the most recent message
                        cache.message = msg;

                        // Forward the message if it is far enough in the past
                        forward = (cache.last_sent + period) <= now;

                        // If forwarding, update the time we last sent to now
                        if (forward) {
                            cache.last_sent = now;
                        }
                    }

                    // Forward the message
                    if (forward) {
                        reactor.powerplant.emit_shared<NetworkScope>(msg, target, config.reliable);
                        NUClear::log<NUClear::LogLevel::TRACE>("Forwarding local `", type, "` message to ", target);
                    }
                });

            double rate = config.rate_limit > 0 ? config.rate_limit : std::numeric_limits<double>::infinity();
            reactor.log<NUClear::LogLevel::INFO>(
                fmt::format("{} -> {} ({}, {}/s)", type, target, config.reliable ? "reliable" : "unreliable", rate));
        }

        /// Bind the reaction for forwarding network messages as local messages
        void bind_inbound_reaction() {
            reaction_handle_in =
                reactor.on<Network<T>, Inline::ALWAYS>().then([this](const std::shared_ptr<const T>& msg) {
                    last_inbound_msg = msg.get();
                    // The message needs to be emitted as a non-const shared pointer so this const cast is necessary to
                    // avoid copying the message. This is safe as the message is not being modified, it's just being
                    // passed from NUClear back to NUClear with a different emit scope.
                    reactor.powerplant.emit_shared<LocalScope>(std::const_pointer_cast<T>(msg));
                    NUClear::log<NUClear::LogLevel::TRACE>("Forwarding network `", type, "` message as local emit");
                });

            reactor.log<NUClear::LogLevel::INFO>(fmt::format("{} <- {}", type, target));
        }

        /// A map of message subtype IDs to their scheduled messages
        std::map<uint32_t, CachedMessage> caches;
        /// The mutex for the caches
        std::mutex mutex;

        /// The reactor that manages this forwarder
        NetworkForwarder& reactor;
        /// The target NUClearNet client to forward messages to and/or from
        std::string target;
        /// The name of the message type to forward
        std::string type;
        /// The configuration for this forwarder
        Config config;
        /// The period implied by the configuration
        NUClear::clock::duration period{};

        /// The NUClear reaction which forwards network messages as local messages
        NUClear::threading::ReactionHandle reaction_handle_in;
        /// The NUClear reaction which forwards local messages as network messages
        NUClear::threading::ReactionHandle reaction_handle_out;

        /// The most recent network message that was forwarded locally
        static thread_local const T* last_inbound_msg;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
    };

    template <typename T>
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    thread_local const T* MessageForwarder<T>::last_inbound_msg = nullptr;

}  // namespace module::network

#endif  // MODULE_NETWORK_NETWORK_FORWARDER_MESSAGE_FORWARDER_HPP