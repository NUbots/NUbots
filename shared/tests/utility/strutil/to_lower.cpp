#include <catch2/catch_test_macros.hpp>

#include "utility/strutil/strutil.hpp"

SCENARIO("Converting string to lowercase", "[utility][strutil][to_lower]") {
    GIVEN("An empty string") {
        std::string input{};
        WHEN("Converting to lowercase") {
            std::string result = utility::strutil::to_lower(input);
            THEN("The result should be an empty string") {
                CHECK(result.empty());
            }
        }
    }

    GIVEN("A string with uppercase letters") {
        std::string input = "HELLO";
        WHEN("Converting to lowercase") {
            std::string result = utility::strutil::to_lower(input);
            THEN("The result should contain only lowercase letters") {
                CHECK(result == "hello");
            }
        }
    }

    GIVEN("A string with lowercase letters") {
        std::string input = "hello";
        WHEN("Converting to lowercase") {
            std::string result = utility::strutil::to_lower(input);
            THEN("The result should remain the same") {
                CHECK(result == "hello");
            }
        }
    }

    GIVEN("A string with mixed case letters") {
        std::string input = "Hello";
        WHEN("Converting to lowercase") {
            std::string result = utility::strutil::to_lower(input);
            THEN("The result should have uppercase letters converted to lowercase") {
                CHECK(result == "hello");
            }
        }
    }

    GIVEN("A string with non-alphabetic characters") {
        std::string input = "Hello, World!";
        WHEN("Converting to lowercase") {
            std::string result = utility::strutil::to_lower(input);
            THEN("The result should have uppercase letters converted to lowercase") {
                CHECK(result == "hello, world!");
            }
        }
    }
}
