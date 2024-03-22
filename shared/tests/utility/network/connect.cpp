#include "utility/network/connect.hpp"

#include <arpa/inet.h>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <fcntl.h>
#include <iostream>
#include <netinet/in.h>
#include <nuclear>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

std::pair<std::thread, int> create_tcp_server(const sa_family_t& family) {

    std::mutex mutex;
    std::condition_variable cv;
    std::unique_lock<std::mutex> lock(mutex);
    /// Volatile as it's accessed in both threads
    volatile uint16_t bound_port = 0;

    // Start a  TCP server that reverses 10 characters and sends them back in reverse order
    std::thread server_thread([&] {
        // Create a TCP socket
        utility::file::FileDescriptor server_fd = ::socket(family, SOCK_STREAM, IPPROTO_TCP);

        // Bind the socket to a local address and port
        if (family == AF_INET) {
            struct sockaddr_in server_addr {};
            server_addr.sin_family      = AF_INET;
            server_addr.sin_addr.s_addr = INADDR_ANY;
            server_addr.sin_port        = 0;  // Use an ephemeral port
            if (::bind(server_fd.get(), reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
                throw std::system_error(errno, std::system_category(), "Unable to bind to port");
            }

            // Get the bound port
            socklen_t addr_len = sizeof(server_addr);
            if (::getsockname(server_fd.get(), reinterpret_cast<sockaddr*>(&server_addr), &addr_len) < 0) {
                throw std::system_error(errno, std::system_category(), "Unable to get socket name");
            }
            bound_port = ::ntohs(server_addr.sin_port);
        }
        else if (family == AF_INET6) {
            struct sockaddr_in6 server_addr {};
            server_addr.sin6_family = AF_INET6;
            server_addr.sin6_addr   = IN6ADDR_ANY_INIT;
            server_addr.sin6_port   = 0;  // Use an ephemeral port
            if (::bind(server_fd.get(), reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
                throw std::system_error(errno, std::system_category(), "Unable to bind to port");
            }

            // Get the bound port
            socklen_t addr_len = sizeof(server_addr);
            if (::getsockname(server_fd.get(), reinterpret_cast<sockaddr*>(&server_addr), &addr_len) < 0) {
                throw std::system_error(errno, std::system_category(), "Unable to get socket name");
            }
            bound_port = ::ntohs(server_addr.sin6_port);
        }

        // Start listening for connections
        if (::listen(server_fd.get(), 1) < 0) {
            throw std::system_error(errno, std::system_category(), "Unable to listen on socket");
        }

        // Notify the cv that it can read the bound port
        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.notify_all();
        }

        // Accept a connection and reverse the first 10 characters received
        std::array<char, 10> buffer{};
        utility::file::FileDescriptor client_fd(::accept(server_fd.get(), nullptr, nullptr),
                                                [](int fd) { ::shutdown(fd, SHUT_RDWR); });

        ::read(client_fd.get(), buffer.data(), buffer.size());
        std::reverse(buffer.begin(), buffer.end());
        ::write(client_fd.get(), buffer.data(), buffer.size());
    });

    // Wait for the server to start up
    cv.wait_for(lock, std::chrono::seconds(1));

    return {std::move(server_thread), bound_port};
}

TEST_CASE("connect() connects to a TCP server and sends data", "[network][connect]") {

    // Number of attempts to send before giving up
    static constexpr int attempts = 10;

    // Test that this works for ipv4, ipv6 and hostnames
    std::string host = GENERATE("127.0.0.1", "localhost");  // ::1 ipv6 does not work in docker

    INFO("Testing connection to " << host);

    // Work out what kind of TCP server we need and create it
    auto sock = NUClear::util::network::resolve(host, 0);

    INFO("Using " << (sock.sock.sa_family == AF_INET ? "ipv4" : "ipv6") << " socket");

    auto [thread, port] = create_tcp_server(sock.sock.sa_family);

    INFO("Bound to port " << port);

    // Connect to the local TCP server
    utility::file::FileDescriptor fd;
    REQUIRE_NOTHROW(fd = utility::network::connect(host, port));

    // Write until it's all sent
    ssize_t bytes_written = 0;
    std::string send      = "0123456789";
    for (int i = 0; i < attempts && bytes_written < std::ssize(send); ++i) {
        ssize_t w = ::write(fd.get(), send.data() + bytes_written, send.size() - bytes_written);
        if (w > 0) {
            bytes_written += w;
        }
        if (bytes_written < std::ssize(send)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    REQUIRE(bytes_written == std::ssize(send));

    // Read until it's all received
    ssize_t bytes_read = 0;
    std::array<char, 10> recv{};
    for (int i = 0; i < attempts && bytes_read < std::ssize(recv); ++i) {
        ssize_t r = ::read(fd.get(), recv.data() + bytes_read, recv.size() - bytes_read);
        if (r > 0) {
            bytes_read += r;
        }
        if (bytes_read < std::ssize(recv)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    REQUIRE(bytes_read == std::ssize(recv));

    // Verify that the response is the reverse of the sent data
    std::string response(recv.data(), bytes_read);
    REQUIRE(response == "9876543210");

    // Wait for the other thread to finish
    REQUIRE_NOTHROW(thread.join());
}
