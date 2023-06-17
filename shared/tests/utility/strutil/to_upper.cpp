#include <catch2/catch_test_macros.hpp>

#include "utility/strutil/strutil.hpp"

SCENARIO("Converting string to uppercase", "[utility][strutil][to_upper]") {
    GIVEN("An empty string") {
        std::string input{};
        WHEN("Converting to uppercase") {
            std::string result = utility::strutil::to_upper(input);
            THEN("The result should be an empty string") {
                CHECK(result.empty());
            }
        }
    }

    GIVEN("A string with lowercase letters") {
        std::string input = "hello";
        WHEN("Converting to uppercase") {
            std::string result = utility::strutil::to_upper(input);
            THEN("The result should contain only uppercase letters") {
                CHECK(result == "HELLO");
            }
        }
    }

    GIVEN("A string with uppercase letters") {
        std::string input = "HELLO";
        WHEN("Converting to uppercase") {
            std::string result = utility::strutil::to_upper(input);
            THEN("The result should remain the same") {
                CHECK(result == "HELLO");
            }
        }
    }

    GIVEN("A string with mixed case letters") {
        std::string input = "Hello";
        WHEN("Converting to uppercase") {
            std::string result = utility::strutil::to_upper(input);
            THEN("The result should have lowercase letters converted to uppercase") {
                CHECK(result == "HELLO");
            }
        }
    }

    GIVEN("A string with non-alphabetic characters") {
        std::string input = "Hello, World!";
        WHEN("Converting to uppercase") {
            std::string result = utility::strutil::to_upper(input);
            THEN("The result should remain the same") {
                CHECK(result == "HELLO, WORLD!");
            }
        }
    }
}
