#include "MCPServer.hpp"

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>
#include <mcp/mcp.hpp>

#include "extension/Configuration.hpp"

namespace module::network {

    using extension::Configuration;

    namespace {

        // Everything here uses only the reflection API exported by libnuclear_message. Pulling in
        // protobuf's json_util instead would statically embed a second copy of the protobuf runtime in
        // this module's library (json_util is not exported by libnuclear_message), and the two copies
        // corrupt each other's descriptor state at runtime.

        std::string base64_encode(const std::string& in) {
            static constexpr char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
            std::string out;
            out.reserve(((in.size() + 2) / 3) * 4);
            for (size_t i = 0; i < in.size(); i += 3) {
                uint32_t chunk = uint32_t(uint8_t(in[i])) << 16;
                if (i + 1 < in.size()) {
                    chunk |= uint32_t(uint8_t(in[i + 1])) << 8;
                }
                if (i + 2 < in.size()) {
                    chunk |= uint32_t(uint8_t(in[i + 2]));
                }
                out.push_back(alphabet[(chunk >> 18) & 0x3F]);
                out.push_back(alphabet[(chunk >> 12) & 0x3F]);
                out.push_back(i + 1 < in.size() ? alphabet[(chunk >> 6) & 0x3F] : '=');
                out.push_back(i + 2 < in.size() ? alphabet[chunk & 0x3F] : '=');
            }
            return out;
        }

        nlohmann::json message_to_json(const google::protobuf::Message& msg);

        nlohmann::json field_to_json(const google::protobuf::Message& msg,
                                     const google::protobuf::Reflection& refl,
                                     const google::protobuf::FieldDescriptor* field,
                                     const int& index) {
            using google::protobuf::FieldDescriptor;
            const bool repeated = field->is_repeated();
            switch (field->cpp_type()) {
                case FieldDescriptor::CPPTYPE_INT32:
                    return repeated ? refl.GetRepeatedInt32(msg, field, index) : refl.GetInt32(msg, field);
                case FieldDescriptor::CPPTYPE_INT64:
                    return repeated ? refl.GetRepeatedInt64(msg, field, index) : refl.GetInt64(msg, field);
                case FieldDescriptor::CPPTYPE_UINT32:
                    return repeated ? refl.GetRepeatedUInt32(msg, field, index) : refl.GetUInt32(msg, field);
                case FieldDescriptor::CPPTYPE_UINT64:
                    return repeated ? refl.GetRepeatedUInt64(msg, field, index) : refl.GetUInt64(msg, field);
                case FieldDescriptor::CPPTYPE_DOUBLE:
                    return repeated ? refl.GetRepeatedDouble(msg, field, index) : refl.GetDouble(msg, field);
                case FieldDescriptor::CPPTYPE_FLOAT:
                    return repeated ? refl.GetRepeatedFloat(msg, field, index) : refl.GetFloat(msg, field);
                case FieldDescriptor::CPPTYPE_BOOL:
                    return repeated ? refl.GetRepeatedBool(msg, field, index) : refl.GetBool(msg, field);
                case FieldDescriptor::CPPTYPE_ENUM:
                    return (repeated ? refl.GetRepeatedEnum(msg, field, index) : refl.GetEnum(msg, field))->name();
                case FieldDescriptor::CPPTYPE_STRING: {
                    std::string s = repeated ? refl.GetRepeatedString(msg, field, index) : refl.GetString(msg, field);
                    return field->type() == FieldDescriptor::TYPE_BYTES ? nlohmann::json(base64_encode(s))
                                                                        : nlohmann::json(std::move(s));
                }
                case FieldDescriptor::CPPTYPE_MESSAGE:
                    return message_to_json(repeated ? refl.GetRepeatedMessage(msg, field, index)
                                                    : refl.GetMessage(msg, field));
            }
            return nullptr;
        }

        // Convert a message to JSON, field names as written in the .proto file, only fields that are set.
        // Map fields come out as arrays of {key, value} objects rather than JSON objects.
        nlohmann::json message_to_json(const google::protobuf::Message& msg) {
            const auto* refl = msg.GetReflection();
            std::vector<const google::protobuf::FieldDescriptor*> fields;
            refl->ListFields(msg, &fields);

            auto out = nlohmann::json::object();
            for (const auto* field : fields) {
                if (field->is_repeated()) {
                    auto arr    = nlohmann::json::array();
                    const int n = refl->FieldSize(msg, field);
                    for (int i = 0; i < n; ++i) {
                        arr.push_back(field_to_json(msg, *refl, field, i));
                    }
                    out[field->name()] = std::move(arr);
                }
                else {
                    out[field->name()] = field_to_json(msg, *refl, field, -1);
                }
            }
            return out;
        }

    }  // namespace

    MCPServer::MCPServer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // Load the message descriptors and register the message buffering reactions
        register_descriptors();
        register_handles();

        on<Configuration>("MCPServer.yaml").then([this](const Configuration& config) {
            this->log_level = config["log_level"].as<NUClear::LogLevel>();

            cfg.host                  = config["host"].as<std::string>();
            cfg.port                  = config["port"].as<int>();
            cfg.path                  = config["path"].as<std::string>();
            cfg.allowed_origins       = config["allowed_origins"].as<std::vector<std::string>>();
            cfg.messages              = config["messages"].as<std::map<std::string, bool>>();
            cfg.max_buffered_messages = config["max_buffered_messages"].as<size_t>();

            // Enable buffering for the message types requested by the configuration
            for (const auto& [name, enabled] : cfg.messages) {
                if (handles.contains(name)) {
                    handles[name].enable(enabled);
                }
                else {
                    log<WARN>("MCP server does not know about the message type", name);
                }
            }
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
            std::string accessible{};
            for (const auto& [name, enabled] : cfg.messages) {
                if (enabled) {
                    accessible += (accessible.empty() ? "" : ", ") + name;
                }
            }
            log<INFO>("MCP server accessible messages:", accessible.empty() ? "(none)" : accessible);
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
        server.tool("read_last_messages",
                    nlohmann::json{
                        {"type", "object"},
                        {"properties", nlohmann::json::object()},
                    },
                    [this](const nlohmann::json&) -> mcp::CallToolResult {
                        // Take the buffered messages, leaving the buffer empty for new ones
                        std::deque<BufferedMessage> messages{};
                        {
                            std::lock_guard<std::mutex> lock(buffer_mutex);
                            std::swap(messages, buffer);
                        }
                        log<DEBUG>("read_last_messages called: I returned", messages.size(), "messages");

                        // Dump the messages as a JSON array, oldest first
                        nlohmann::json out = nlohmann::json::array();
                        for (const auto& m : messages) {
                            auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                    m.timestamp.time_since_epoch())
                                                    .count();
                            nlohmann::json entry{
                                {"type", m.type},
                                {"timestamp_ms", timestamp_ms},
                                {"data", nullptr},
                            };

                            // The generated C++ protobuf classes are lite (no reflection), so decode the payload
                            // into a full dynamic message built from the embedded descriptors
                            const auto* descriptor = descriptor_pool.FindMessageTypeByName(m.pb_name);
                            if (descriptor != nullptr) {
                                std::unique_ptr<google::protobuf::Message> pb(
                                    message_factory.GetPrototype(descriptor)->New());
                                if (pb->ParseFromArray(m.payload.data(), static_cast<int>(m.payload.size()))) {
                                    entry["data"] = message_to_json(*pb);
                                }
                            }
                            out.push_back(std::move(entry));
                        }
                        return {
                            .content = {mcp::TextContent{.text = out.dump()}},
                        };
                    });
    }

}  // namespace module::network
