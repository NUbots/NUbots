#ifndef MODULE_NETWORK_MCPSERVER_HPP
#define MODULE_NETWORK_MCPSERVER_HPP

#include <nuclear>

namespace module::network {

class MCPServer : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

public:
    /// @brief Called by the powerplant to build and setup the MCPServer reactor.
    explicit MCPServer(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::network

#endif  // MODULE_NETWORK_MCPSERVER_HPP
