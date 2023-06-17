#include <catch2/catch_test_macros.hpp>

#include "utility/strutil/strutil.hpp"

SCENARIO("Removing characters from a string", "[utility][strutil][remove_all]") {
    GIVEN("An empty string and empty tokens") {
        std::string input{};
        std::string tokens{};
        WHEN("Removing characters") {
            std::string result = utility::strutil::remove_all(input, tokens);
            THEN("The result should be an empty string") {
                CHECK(result.empty());
            }
        }
    }

    GIVEN("An empty string and non-empty tokens") {
        std::string input{};
        std::string tokens = "abc";
        WHEN("Removing characters") {
            std::string result = utility::strutil::remove_all(input, tokens);
            THEN("The result should be an empty string") {
                CHECK(result.empty());
            }
        }
    }

    GIVEN("A non-empty string and empty tokens") {
        std::string input = "Hello, World!";
        std::string tokens{};
        WHEN("Removing characters") {
            std::string result = utility::strutil::remove_all(input, tokens);
            THEN("The result should be the same as the input string") {
                CHECK(result == "Hello, World!");
            }
        }
    }

    GIVEN("A non-empty string and non-empty tokens") {
        std::string input  = "Hello, World!";
        std::string tokens = "o";
        WHEN("Removing characters") {
            std::string result = utility::strutil::remove_all(input, tokens);
            THEN("The result should be the input string with the specified characters removed") {
                CHECK(result == "Hell, Wrld!");
            }
        }
    }
}
