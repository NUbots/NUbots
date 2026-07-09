#ifndef MODULE_NETWORK_MCPSERVER_HPP
#define MODULE_NETWORK_MCPSERVER_HPP

#include <memory>
#include <mcp/http_server_host.hpp>
#include <nuclear>
#include <string>
#include <vector>

namespace module::network {

class MCPServer : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
        /// @brief Address the MCP endpoint binds to
        std::string host = "127.0.0.1";
        /// @brief Port the MCP endpoint listens on
        int port = 8080;
        /// @brief URL path of the Streamable HTTP endpoint
        std::string path = "/mcp";
        /// @brief Origin allowlist for DNS-rebinding protection
        std::vector<std::string> allowed_origins{};
    } cfg;

    /// @brief The MCP Streamable HTTP host, created on Startup and stopped on Shutdown
    std::unique_ptr<mcp::HttpServerHost> host{};

    /// @brief Register the tools exposed to each MCP session
    void register_tools(mcp::Server& server);

public:
    /// @brief Called by the powerplant to build and setup the MCPServer reactor.
    explicit MCPServer(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::network

#endif  // MODULE_NETWORK_MCPSERVER_HPP
