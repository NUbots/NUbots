#include "MCPServer.hpp"

#include <mcp/mcp.hpp>

#include "extension/Configuration.hpp"

namespace module::network {

    using extension::Configuration;

    MCPServer::MCPServer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("MCPServer.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.host            = config["host"].as<std::string>();
            cfg.port            = config["port"].as<int>();
            cfg.path            = config["path"].as<std::string>();
            cfg.allowed_origins = config["allowed_origins"].as<std::vector<std::string>>();
        });

        on<Startup>().then([this] {
            host = std::make_unique<mcp::HttpServerHost>(mcp::Implementation{.name = "nubots-mcp", .version = "1.0.0"},
                                                         mcp::HttpServerHost::Options{
                                                             .host            = cfg.host,
                                                             .port            = cfg.port,
                                                             .path            = cfg.path,
                                                             .allowed_origins = cfg.allowed_origins,
                                                         },
                                                         [this](mcp::Server& server) { register_tools(server); });

            host->start();
            log<INFO>("MCP server listening on", cfg.host, "port", host->port(), "path", cfg.path);
        });

        on<Shutdown>().then([this] {
            if (host != nullptr) {
                host->stop();
                host.reset();
            }
        });
    }

    void MCPServer::register_tools(mcp::Server& server) {
        server.tool("get_status",
                    nlohmann::json{
                        {"type", "object"},
                        {"properties", nlohmann::json::object()},
                    },
                    [this](const nlohmann::json&) -> mcp::CallToolResult {
                        log<DEBUG>("get_status called: I returned \"She'll be right.\"");
                        return {
                            .content = {mcp::TextContent{.text = "She'll be right."}},
                        };
                    });
    }

}  // namespace module::network
