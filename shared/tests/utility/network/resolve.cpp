#include "utility/network/resolve.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("resolve function returns expected socket address", "[network][resolve]") {

    SECTION("IPv4 address") {
        std::string address = "127.0.0.1";
        uint16_t port       = 80;

        auto result = utility::network::resolve(address, port);

        REQUIRE(result.sock.sa_family == AF_INET);
        REQUIRE(ntohs(result.ipv4.sin_port) == port);
        REQUIRE(ntohl(result.ipv4.sin_addr.s_addr) == INADDR_LOOPBACK);
    }

    SECTION("IPv6 address") {
        std::string address = "::1";
        uint16_t port       = 80;

        auto result = utility::network::resolve(address, port);

        REQUIRE(result.sock.sa_family == AF_INET6);
        REQUIRE(ntohs(result.ipv6.sin6_port) == port);
        REQUIRE(ntohl(result.ipv6.sin6_addr.s6_addr32[0]) == 0);
        REQUIRE(ntohl(result.ipv6.sin6_addr.s6_addr32[1]) == 0);
        REQUIRE(ntohl(result.ipv6.sin6_addr.s6_addr32[2]) == 0);
        REQUIRE(ntohl(result.ipv6.sin6_addr.s6_addr32[3]) == 1);
    }

    SECTION("Hostname") {
        std::string address = "localhost";
        uint16_t port       = 80;

        auto result = utility::network::resolve(address, port);

        // Check that the returned socket address matches the expected address and port
        REQUIRE((result.sock.sa_family == AF_INET || result.sock.sa_family == AF_INET6));

        // Localhost could return an ipv4 or ipv6 address
        if (result.sock.sa_family == AF_INET) {
            REQUIRE(ntohs(result.ipv4.sin_port) == port);
            REQUIRE(ntohl(result.ipv4.sin_addr.s_addr) == INADDR_LOOPBACK);
        }
        else {
            REQUIRE(ntohs(result.ipv6.sin6_port) == port);
            REQUIRE(ntohl(result.ipv6.sin6_addr.s6_addr32[0]) == 0);
            REQUIRE(ntohl(result.ipv6.sin6_addr.s6_addr32[1]) == 0);
            REQUIRE(ntohl(result.ipv6.sin6_addr.s6_addr32[2]) == 0);
            REQUIRE(ntohl(result.ipv6.sin6_addr.s6_addr32[3]) == 1);
        }
    }

    SECTION("IPv4 address with leading zeros") {
        std::string address = "127.000.000.001";
        uint16_t port       = 80;

        auto result = utility::network::resolve(address, port);

        REQUIRE(result.sock.sa_family == AF_INET);
        REQUIRE(ntohs(result.ipv4.sin_port) == port);
        REQUIRE(ntohl(result.ipv4.sin_addr.s_addr) == INADDR_LOOPBACK);
    }

    SECTION("IPv6 address with mixed case letters") {
        std::string address = "2001:0DB8:Ac10:FE01:0000:0000:0000:0000";
        uint16_t port       = 80;

        auto result = utility::network::resolve(address, port);

        REQUIRE(result.sock.sa_family == AF_INET6);
        REQUIRE(ntohs(result.ipv6.sin6_port) == port);
        REQUIRE(ntohl(result.ipv6.sin6_addr.s6_addr32[0]) == 0x20010db8);
        REQUIRE(ntohl(result.ipv6.sin6_addr.s6_addr32[1]) == 0xac10fe01);
        REQUIRE(ntohl(result.ipv6.sin6_addr.s6_addr32[2]) == 0);
        REQUIRE(ntohl(result.ipv6.sin6_addr.s6_addr32[3]) == 0);
    }

    SECTION("Hostname with valid IPv4 address") {
        std::string address = "www.google.com";
        uint16_t port       = 80;

        auto result = utility::network::resolve(address, port);

        REQUIRE(result.sock.sa_family == AF_INET);
        REQUIRE(ntohs(result.ipv4.sin_port) == port);
        REQUIRE(ntohl(result.ipv4.sin_addr.s_addr) != 0);
    }

    SECTION("Hostname with valid IPv6 address") {
        std::string address = "ipv6.google.com";
        uint16_t port       = 80;

        auto result = utility::network::resolve(address, port);

        REQUIRE(result.sock.sa_family == AF_INET6);
        REQUIRE(ntohs(result.ipv6.sin6_port) == port);

        // Check if all compoments are zero
        bool nonzero = false;
        for (int i = 0; i < 4; i++) {
            nonzero |= (ntohl(result.ipv6.sin6_addr.s6_addr32[i]) != 0);
        }

        REQUIRE(nonzero);
    }

    SECTION("Invalid address") {
        std::string address = "notahost";
        uint16_t port       = 12345;

        // Check that the function throws a std::runtime_error with the appropriate message
        REQUIRE_THROWS(utility::network::resolve(address, port));
    }
}
