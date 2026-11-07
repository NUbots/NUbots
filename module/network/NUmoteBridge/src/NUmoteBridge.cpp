#include "NUmoteBridge.hpp"

#include <array>
#include <string>
#include <tuple>
#include <unistd.h>

#include "extension/Configuration.hpp"

namespace module::network {

    using extension::Configuration;

    NUmoteBridge::NUmoteBridge(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("NUmoteBridge.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();

            const int new_port = config["port"].as<int>();
            if (listener.enabled() && new_port == cfg.port) {
                return;
            }
            cfg.port = new_port;

            // The listener is the only thing running while no viewer is connected.


            listener.unbind();

            auto result =
                on<TCP>(cfg.port).then([this](const TCP::Connection& connection) { on_connect(connection.fd); });
            listener = std::get<0>(result);


            log<INFO>("NUmoteBridge listening on port", cfg.port);
        });
    }

    void NUmoteBridge::on_connect(int fd) {
        clients.insert(fd);
        log<INFO>("Viewer connected, fd", fd, "- clients:", clients.size());

        const std::string hello = "NUmoteBridge hello\n";
        ::write(fd, hello.data(), hello.size());

        // Per-connection reaction. Unbinds itself when the viewer goes away.
        auto handle = std::make_shared<ReactionHandle>();
        *handle     = on<IO>(fd, IO::READ | IO::CLOSE | IO::ERROR).then([this, fd, handle](const IO::Event& event) {
            if (event.events & IO::READ) {
                std::array<char, 1024> buf{};
                const ssize_t n = ::read(fd, buf.data(), buf.size());
                if (n > 0) {
                    log<INFO>("Received", n, "bytes from fd", fd);
                    return;
                }
                // n <= 0 means the peer closed the connection
            }
            handle->unbind();
            on_disconnect(fd);
        });
    }

    void NUmoteBridge::on_disconnect(int fd) {
        ::close(fd);
        clients.erase(fd);
        log<INFO>("Viewer disconnected, fd", fd, "- clients:", clients.size());
    }

}  // namespace module::network
