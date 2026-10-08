#ifndef MODULE_NETWORK_MCPSERVER_HPP
#define MODULE_NETWORK_MCPSERVER_HPP

#include <mcp/http_server_host.hpp>
#include <memory>
#include <mutex>
#include <nuclear>
#include <string>
#include <vector>

#include "message/input/Image.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"

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
            /// @brief Boolean for allowing Claude to run random commands with no oversight
            bool allow_ace = false;
        } cfg;

        /// @brief The MCP Streamable HTTP host, created on Startup and stopped on Shutdown
        std::unique_ptr<mcp::HttpServerHost> host{};

        /// @brief Guards last_image, which is written from the camera reaction and read from MCP tool calls
        std::mutex image_mutex;
        /// @brief The most recent raw camera image, if one has been seen yet
        std::shared_ptr<const message::input::Image> last_image{};

        /// @brief Guards last_sensors, which is written from the Sensors reaction and read from MCP tool calls
        std::mutex sensors_mutex;
        /// @brief The most recent Sensors message if one has been seen yet
        std::shared_ptr<const message::input::Sensors> last_sensors{};

        /// @brief Guards last_field, which is written from the Field reaction and read from MCP tool calls
        std::mutex field_mutex;
        /// @brief The most recent Field localisation message if one has been seen yet
        std::shared_ptr<const message::localisation::Field> last_field{};

        /// @brief Register the tools exposed to each MCP session
        void register_tools(mcp::Server& server);

    public:
        /// @brief Called by the powerplant to build and setup the MCPServer reactor.
        explicit MCPServer(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::network

#endif  // MODULE_NETWORK_MCPSERVER_HPP
