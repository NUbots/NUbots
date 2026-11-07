#ifndef MODULE_NETWORK_NUMOTEBRIDGE_HPP
#define MODULE_NETWORK_NUMOTEBRIDGE_HPP

#include <nuclear>
#include <set>

namespace module::network {

    class NUmoteBridge : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            int port = 51120;
        } cfg;

        // Listens for NUmote connections. Rebound if the port changes
        ReactionHandle listener{};

        // file descriptors of connected viewers
        std::set<int> clients;

        void on_connect(int fd);
        void on_disconnect(int fd);


    public:
        /// @brief Called by the powerplant to build and setup the NUmoteBridge reactor.
        explicit NUmoteBridge(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::network

#endif  // MODULE_NETWORK_NUMOTEBRIDGE_HPP
